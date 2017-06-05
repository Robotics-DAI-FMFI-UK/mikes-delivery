#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "config_mikes.h"
#include "util.h"
#include "mcl.h"
#include "astar.h"
#include "base_module.h"
#include "pose.h"

#define POSTERIOR_CONST 0.95

#define MCL_UPDATE_MAX_PERIOD  10

#define MAX_CLUSTER_COUNT  HYPO_COUNT
#define MAX_SAME_CLUSTER_DISTANCE_SQ  10000
#define MAX_SAME_CLUSTER_ANGLE_DISTANCE 120

pthread_mutex_t lidar_mcl_lock;

hypo_t hypo[2][HYPO_COUNT];
hypo_t current_hypo[HYPO_COUNT];
point poly_i[NUMBER_OF_VERTICES_I]; // pavilon I
point poly_a[NUMBER_OF_VERTICES_A]; // atrium
point poly_h[NUMBER_OF_VERTICES_H]; // miestnosti H3 a H6

line lines[NUMBER_OF_VERTICES]; //vsetky ciary z mapy

int activeHypo = 0;

short hypo_cluster[HYPO_COUNT];
unsigned short cluster_count;

int cluster_representant_x[MAX_CLUSTER_COUNT];   // indexed by cluster id
int cluster_representant_y[MAX_CLUSTER_COUNT];
double cluster_representant_alpha[MAX_CLUSTER_COUNT];

double cluster_weight_sum[MAX_CLUSTER_COUNT];
int cluster_hypo_count[MAX_CLUSTER_COUNT];

double cluster_x;
double cluster_y;
double cluster_alpha;

pose_type old_pose;

int point_in_polygon(int n_vert, point *vertices, double test_x, double test_y)
{
   int i, j = 0;
   bool c = false;
   for (i = 0, j = n_vert - 1; i < n_vert; j = i++) {
      if ( ((vertices[i].y > test_y) != (vertices[j].y > test_y))
          && (test_x < (vertices[j].x - vertices[i].x) * (test_y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x) )
      {
         c = !c;
      }
   }
   return c;
}

int is_in_corridor(int cx, int cy) {
   if (!point_in_polygon(NUMBER_OF_VERTICES_A, poly_a, cx, cy)
       && !point_in_polygon(NUMBER_OF_VERTICES_H, poly_h, cx, cy) 
       && point_in_polygon(NUMBER_OF_VERTICES_I, poly_i, cx, cy))
      return 1;
   else
      return 0;
}

const double epsilon = 0.000001;

void swap(double *a, double *b)
{
	double p = *a;
	*a = *b;
	*b = p;
}

void reduce(double x1, double x2, double *x3, double *x4)
{
	if (*x3 > x1)
	{
		if (*x3 < x2) *x3 = x1;
		else *x3 -= (x2 - x1);
	}
	if (*x4 > x1)
	{
		if (*x4 < x2) *x4 = x1;
		else *x4 -= (x2 - x1);
	}
}

int check_bouding_box_intersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
{
	if (x2 < x1) swap(&x1, &x2);
	if (y2 < y1) swap(&y1, &y2);
	if (x4 < x3) swap(&x3, &x4);
	if (y4 < y3) swap(&y3, &y4);
	
	reduce(x1, x2, &x3, &x4);
	reduce(y1, y2, &y3, &y4);
	
	if((x3 <= x1) && (x4 >= x1) &&
	   (y3 <= y1) && (y4 >= y1))
		return 1;
	else
		return 0;
}

double line_intersection(
                         double x1, double y1,
                         double x2, double y2,
                         double x3, double y3,
                         double x4, double y4,
                         double *X, double *Y)
{	

   if (!check_bouding_box_intersect(x1,y1,x2,y2,x3,y3,x4,y4))
      return -3;
      
   double A1 = y2 - y1;
   double B1 = x1 - x2;
   double C1 = A1 * x1 + B1 * y1;
   
   double A2 = y4 - y3;
   double B2 = x3 - x4;
   double C2 = A2 * x3 + B2 * y3;
   
   double det = A1 * B2 - A2 * B1;
   if (det == 0) {
      //Lines are parallel
      return -1;
   } else {
      double x = (B2*C1 - B1*C2)/det;
      double y = (A1*C2 - A2*C1)/det;
      if ((fmin(x1,x2) < (x + epsilon))
          && (x < (fmax(x1,x2) + epsilon))
          && (fmin(y1,y2) < (y + epsilon))
          && (y < (fmax(y1,y2) + epsilon))
          && (fmin(x3,x4) < (x + epsilon))
          && (x < (fmax(x3,x4) + epsilon))
          && (fmin(y3,y4) < (y + epsilon))
          && (y < (fmax(y3,y4) + epsilon)))
      {
         *X = x;
         *Y = y;
         return 1;
      } else
         return -2;
   }
}

int get_line_intersection(double x1, double y1, double x2, double y2) {
   
   double nx = 0;
   double ny = 0;
   
   for (int i = 0; i < NUMBER_OF_VERTICES; ++i)
   {
		int li = line_intersection(x1, y1, x2, y2, lines[i].x1, lines[i].y1, lines[i].x2, lines[i].y2, &nx, &ny);
		if (li > 0)
			return 1;
   }
   
   return 0;
}

static double sqrt_2PI;

double get_sensor_model(double d, double x) {
   
   double sigma = sqrt(2); 
   double scale_input = 40;  // in cm
   
   d /= scale_input;
   x /= scale_input;
   
   double exponent = (x - d) * (x - d) / (2 * sigma * sigma);
   double result = exp(-exponent) / (sigma * sqrt_2PI);
   
   return result;
}

void get_mcl_data(hypo_t *buffer)
{
   pthread_mutex_lock(&lidar_mcl_lock);
   memcpy(buffer, current_hypo, sizeof(hypo_t) * HYPO_COUNT);
   pthread_mutex_unlock(&lidar_mcl_lock);
}

double generateGaussianNoise(double mu, double sigma)
{
   const double epsilon = 0.000000001;
   const double two_pi = 2.0*3.14159265358979323846;
   
   static double z0, z1;
   static int generate;
   generate = 1-generate;
   
   if (!generate)
      return z1 * sigma + mu;
   
   double u1, u2;
   do
   {
      u1 = rand() * (1.0 / RAND_MAX);
      u2 = rand() * (1.0 / RAND_MAX);
   }
   while ( u1 <= epsilon );
   
   z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
   z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
   return z0 * sigma + mu;
}

static double max_dist = 600*600;

double get_min_intersection_dist(double x1, double y1, double alpha) {
   
   double lm = max_dist;
   double min_dist = max_dist;
   double dist = max_dist;
   
   double x2 = x1 + lm*sin(alpha);
   double y2 = y1 - lm*cos(alpha);
   
   double nx = x1;
   double ny = y1;
   
   for (int i = 0; i < NUMBER_OF_VERTICES; ++i) {
	   if (line_intersection(x1, y1, x2, y2, lines[i].x1, lines[i].y1, lines[i].x2, lines[i].y2, &nx, &ny) > 0)
	   {
		   dist = (nx - x1)*(nx - x1) + (ny - y1)*(ny - y1);
		   if (dist < min_dist)
			   min_dist = dist;
	   }
	   
   }

   return sqrt(min_dist);
   
}

#define KLARGEST   (HYPO_COUNT * 14 / 15)
#define CLUSTERED_HYPOTHESES_RATIO  0.7

double largest_values[KLARGEST];

double find_kth_largest_hypothesis_weight()
{
   for (int i = 0; i < KLARGEST; i++)
      largest_values[i] = hypo[activeHypo][i].w;
   for (int i = 0; i < KLARGEST; i++)
   {
      int minimal = i;
      for (int j = i + 1; j < KLARGEST; j++)
         if (largest_values[j] < largest_values[minimal])
            minimal = j;
      double tmp = largest_values[i];
      largest_values[i] = largest_values[minimal];
      largest_values[minimal] = tmp;
   }
   for (int i = KLARGEST; i < HYPO_COUNT; i++)
   {
     int better_than = 0;
     double weight = hypo[activeHypo][i].w;
     while ((better_than < KLARGEST) && (weight > largest_values[better_than])) 
       better_than++;
     if (better_than > 0)
     {
        for (int j = 0; j < better_than - 1; j++)
          largest_values[j] = largest_values[j + 1];
        largest_values[better_than - 1] = weight;
     }
  }
  return largest_values[0];
}


void estimate_pose()
{
   cluster_count = 0;
   double weight_threshold = CLUSTERED_HYPOTHESES_RATIO * find_kth_largest_hypothesis_weight();

   int hypotheses_considered = 0;

   for (int i = 0; i < HYPO_COUNT; i++)
   {
      double weight = hypo[activeHypo][i].w;
      if (weight < weight_threshold) 
      {
          hypo_cluster[i] = -1;
          continue;
      }
      hypotheses_considered++;

      double x = hypo[activeHypo][i].x;
      double y = hypo[activeHypo][i].y;
      double alpha = hypo[activeHypo][i].alpha;
      int belongs_to_cluster = -1;
      for (int clus = 0; clus < cluster_count; clus++)
      {
         long dsquared = (cluster_representant_x[clus] - x);
         dsquared *= dsquared;
         long ysq = (cluster_representant_y[clus] - y);
         dsquared =+ ysq * ysq;
         double alphadiff = angle_rad_difference(cluster_representant_alpha[clus], alpha);

         if ((dsquared < MAX_SAME_CLUSTER_DISTANCE_SQ) && (alphadiff < MAX_SAME_CLUSTER_ANGLE_DISTANCE))
         {
                belongs_to_cluster = clus;
                break;
         }
      }
      if (belongs_to_cluster == -1)
      {
         cluster_representant_x[cluster_count] = x;
         cluster_representant_y[cluster_count] = y;
         cluster_representant_alpha[cluster_count] = alpha;
         cluster_weight_sum[cluster_count] = 0.0;
         cluster_hypo_count[cluster_count] = 0;
	 belongs_to_cluster = cluster_count++;
      }
         
      hypo_cluster[i] = belongs_to_cluster;
      cluster_weight_sum[belongs_to_cluster] += weight;
      cluster_hypo_count[belongs_to_cluster]++;
   } 
   mikes_log_val(ML_INFO, "MCL hypotheses considered", hypotheses_considered);

   int best_average_cluster = 0;
   double best_average_cluster_weight = 0.0;

   for (int i = 1; i < cluster_count; i++)
   {
      double average_cluster_weight = cluster_weight_sum[i] / cluster_hypo_count[i];
      if (average_cluster_weight > best_average_cluster_weight)
      {
         best_average_cluster_weight = average_cluster_weight;
         best_average_cluster = i;
      }
   }
        
   double best_cluster_weight_sum = cluster_weight_sum[best_average_cluster];
   cluster_x = 0.0;
   cluster_y = 0.0;
   cluster_alpha = 0.0;

   for (int i = 0; i < HYPO_COUNT; i++)
   {
      if (hypo_cluster[i] != best_average_cluster) continue;

      double weight = hypo[activeHypo][i].w;
      double hypo_contributing_ratio = (weight / best_cluster_weight_sum);
      
      cluster_x += hypo_contributing_ratio * hypo[activeHypo][i].x;
      cluster_y += hypo_contributing_ratio * hypo[activeHypo][i].y;
      cluster_alpha += hypo_contributing_ratio * hypo[activeHypo][i].alpha;
   }

   mikes_log_double2(ML_INFO, "MCL estimated pose at x,y:", cluster_x, cluster_y);
   mikes_log_double(ML_INFO, "                    alpha:", cluster_alpha);
   pose_type new_pose;

   get_pose(&new_pose);

   cluster_x += new_pose.x - old_pose.x;
   cluster_y += new_pose.y - old_pose.y;
   cluster_alpha += angle_rad_difference(old_pose.heading, new_pose.heading);
   //set_pose(cluster_x, cluster_y, cluster_alpha);
}

int mcl_update()
{
   lidar_data_type liddata;

   get_lidar_data(&liddata);
   get_pose(&old_pose);

   double vect_x = fabs(old_pose.x - cluster_x);
   double vect_y = fabs(old_pose.y - cluster_y);

   double dist =  vect_x*vect_x + vect_y*vect_y;
   double traveled =  sqrt(dist); 

   double heading = old_pose.heading - cluster_alpha;

   mikes_log_double(ML_INFO, "MCL New data - traveled:", traveled);
   mikes_log_val(ML_INFO, "MCL New data - heading:", heading);

   long start_mcl_timestamp = usec_time();
   
   activeHypo = 1-activeHypo;
   
//   for(int i = 0; i < HYPO_COUNT; i++){
//      mikes_log_val(ML_DEBUG, "hypo id premove: ", i);
//      mikes_log_val2(ML_DEBUG, "hypo pos: ", hypo[1-activeHypo][i].x,hypo[1-activeHypo][i].y);
//      mikes_log_val2(ML_DEBUG, "hypo v&a: ", hypo[1-activeHypo][i].w*100,hypo[1-activeHypo][i].alpha);
//   }
   
   double max_hypo_prob = 0;
   
   for (int i = 0; i < HYPO_COUNT; i++)
   {
      
      hypo[1-activeHypo][i].x += vect_x;
      hypo[1-activeHypo][i].y -= vect_y;
      hypo[1-activeHypo][i].alpha = rad_normAlpha(hypo[1-activeHypo][i].alpha+heading);
         
      // point outside of the corridor
      if(!is_in_corridor(hypo[1-activeHypo][i].x, hypo[1-activeHypo][i].y)) {
         hypo[1-activeHypo][i].w = 0;
         continue;
      }      
      
      //sensor position
      double possx = hypo[1-activeHypo][i].x + sin(hypo[1-activeHypo][i].alpha) * 7;
      double possy = hypo[1-activeHypo][i].y - cos(hypo[1-activeHypo][i].alpha) * 7;
      
      double w_post = hypo[1-activeHypo][i].w;
      for (int j = 0; j < liddata.count; ++j) 
      {
            uint16_t measured_distance = liddata.distance[j] / 40; // Actual distance = distance_q2 / 4 mm // but we want distance in cm
            if (measured_distance == 0 || (liddata.angle[j] > (120 * 64) && liddata.angle[j] < (240 * 64)))
				continue;
  	        if (measured_distance > 600)
		    measured_distance = 600;
            
            uint16_t angle_64 = liddata.angle[j];
            double angle = (angle_64 / 64.0) / 180.0 * M_PI;
            
            uint16_t computed_distance = (uint16_t) get_min_intersection_dist(possx, possy, angle);
            double smodel = get_sensor_model(computed_distance, measured_distance);
            
	        double w_post_new = w_post * smodel;
	        w_post_new = w_post_new / (double) NORM_PROB_CONST;
	        w_post = POSTERIOR_CONST*w_post + (1-POSTERIOR_CONST)*w_post_new;
      }
         
      mikes_log_double2(ML_DEBUG, "apriori | posterior ", hypo[1-activeHypo][i].w, w_post);
         
      if (w_post > max_hypo_prob)
			 max_hypo_prob = w_post;
			
      hypo[1-activeHypo][i].w = w_post;
   }
   
   for (int i = 0; i < HYPO_COUNT; i++)
   {
	   hypo[1-activeHypo][i].w /= max_hypo_prob;
   }
   
   
//   for(int i = 0; i< HYPO_COUNT; i++){
//      mikes_log_val(ML_DEBUG, "hypo id postmove: ", i);
//      mikes_log_val2(ML_DEBUG, "hypo pos: ", hypo[1-activeHypo][i].x,hypo[1-activeHypo][i].y);
//      mikes_log_val2(ML_DEBUG, "hypo v&a: ", hypo[1-activeHypo][i].w*100,hypo[1-activeHypo][i].alpha);
//   }
   
   // cumulative probability
   double cumP[HYPO_COUNT];
   double last = 0;
   for(int i = 0; i < HYPO_COUNT; i++){
      last += hypo[1-activeHypo][i].w;
      cumP[i] = last;
   }
   
   // generate new particle set
   int i;
   for(i = 0; i < HYPO_COUNT*0.9; i++)
   {
      double next = (double)rand() / (double)RAND_MAX * last;
      for(int j = 0; j < HYPO_COUNT; j++){
         if( next <= cumP[j]){
            hypo[activeHypo][i].x = hypo[1-activeHypo][j].x + generateGaussianNoise(0, 0.03*traveled);
            hypo[activeHypo][i].y = hypo[1-activeHypo][j].y + generateGaussianNoise(0, 0.03*traveled);
            hypo[activeHypo][i].alpha = rad_normAlpha(hypo[1-activeHypo][j].alpha + generateGaussianNoise(0, heading*0.05));
            hypo[activeHypo][i].w = hypo[1-activeHypo][j].w;
            break;
         }
      }
   }
   
   // generate random particles
   for(; i< HYPO_COUNT; i++)
   {
      
      double rand_x = 0;
      double rand_y = 0;
      
      do {
         rand_x = rand() % MAP_W;
         rand_y = rand() % MAP_H;
      } while (!is_in_corridor(rand_x, rand_y));
      
      hypo[activeHypo][i].x = rand_x;
      hypo[activeHypo][i].y = rand_y;
      hypo[activeHypo][i].alpha = rand() / (double)RAND_MAX * 2.0 * M_PI;
      hypo[activeHypo][i].w = 0.01;
      
   }
   
   pthread_mutex_lock(&lidar_mcl_lock);
   memcpy(current_hypo, hypo[activeHypo], sizeof(hypo_t) * HYPO_COUNT);
   pthread_mutex_unlock(&lidar_mcl_lock);
   
   estimate_pose();

   
   long end_mcl_timestamp = usec_time();
   mikes_log_val(ML_DEBUG, "mcl time: ", (end_mcl_timestamp - start_mcl_timestamp) / 1000);
   return 0;
}


void *mcl_thread(void *arg)
{
	int mcl_update_threshold = 100; //minimal number of ticks to make mcl_update
	base_data_type base_data;
	lidar_data_type lidar_data;
	sleep(3);
	
	base_data_type old_base_data;
	get_base_data(&old_base_data);
	get_lidar_data(&lidar_data);
	get_pose(&old_pose);

        int timer_counter = 0;

	// mcl computes in cm
	while (program_runs)
	{
		
		//printf("mcl: counterA %d counterB %d, heading %d\n", base_data.counterA, base_data.counterB, base_data.heading);
		
		if (timer_counter > MCL_UPDATE_MAX_PERIOD) 
		{
			mcl_update();
			timer_counter = 0;
		}
		
		sleep(1);
		timer_counter++;
	}
	
	mikes_log(ML_INFO, "mcl quits.");
	threads_running_add(-1);
	return 0;
}

int init_mcl()
{
   sqrt_2PI = sqrt(2 * M_PI);
   pthread_t t;
   if (pthread_create(&t, 0, mcl_thread, 0) != 0)
   {
      perror("mikes:mcl");
      mikes_log(ML_ERR, "creating mcl thread");
   }
   else threads_running_add(1);

   pthread_mutex_init(&lidar_mcl_lock, 0);
   
   // seed random generator
   time_t tm;
   srand((unsigned) time(&tm));
   
   // get map
   get_lines_from_file("mapa_pavilonu_I.svg", lines);
   get_polygons(poly_i, poly_a, poly_h, lines); 
      
   base_data_type base_data;
   get_base_data(&base_data);
   double heading = base_data.heading / (double)180 * M_PI;
   
   // init primary mcl particles
   for (int i = 0; i < HYPO_COUNT; i++){
      
      double rand_x = 0;
      double rand_y = 0;
      
      do {
         rand_x = rand() % MAP_W;
         rand_y = rand() % MAP_H;
      } while (!is_in_corridor(rand_x, rand_y));
      
      
      hypo[0][i].x = hypo[1][i].x = rand_x;
      hypo[0][i].y = hypo[1][i].y = rand_y;
      hypo[0][i].alpha = hypo[1][i].alpha = rand() / (double)RAND_MAX * 2.0 * M_PI;
      
      double angle_diff = angle_rad_difference(hypo[0][i].alpha, heading);
      
      // incorporationg actual robot heading into primary mcl particles weight
      hypo[0][i].w = hypo[1][i].w = 0.05 + 0.3 * (M_PI - fabs(angle_diff) / M_PI);
      
      //mikes_log_val(ML_INFO, "hypo id: ", i);
      //mikes_log_double2(ML_INFO, "hypo pos: ", hypo[0][i].x,hypo[0][i].y);
      //mikes_log_double2(ML_INFO, "hypo v&a: ", hypo[0][i].w,hypo[0][i].alpha);
   }
   
   return 0;
}
