#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <libxml/xmlreader.h>

#include "mikes_logs.h"
#include "mcl.h"


#define MAP_W 4578
#define MAP_H 4303
#define NUMBER_OF_VERTICES 1000
#define NUMBER_OF_VERTICES_I 101
#define NUMBER_OF_VERTICES_A 49
#define NUMBER_OF_VERTICES_H 13
#define ROOM_I 1
#define ROOM_ATRIUM 2
#define ROOM_H3_H6 3

pthread_mutex_t lidar_mcl_lock;

hypo_t hypo[2][HYPO_COUNT];
point poly_i[NUMBER_OF_VERTICES_I]; // pavilon I
point poly_a[NUMBER_OF_VERTICES_A]; // atrium
point poly_h[NUMBER_OF_VERTICES_H]; // miestnosti H3 a H6

int vert_n_i;
int vert_n_a;
int vert_n_h;

int activeHypo = 0;

int pnpoly(int n_vert, point *vertices, double test_x, double test_y)
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
   if (pnpoly(vert_n_i, poly_i, cx, cy)
       && !pnpoly(vert_n_a, poly_a, cx, cy)
       && !pnpoly(vert_n_h, poly_h, cx, cy))
      return 1;
   else
      return 0;
}

static int parse_element(xmlNode* a_node, point* points, int room)
{
   xmlNode *cur_node = NULL;
   int start_line_id;
   int end_line_id;
   
   switch (room) {
      case ROOM_I:
         start_line_id = 0;
         end_line_id = 100;
         break;
      case ROOM_ATRIUM:
         start_line_id = 101;
         end_line_id = 149;
         break;
      case ROOM_H3_H6:
         start_line_id = 150;
         end_line_id = 162;
         break;
      default:
         mikes_log(ML_ERR, "Wrong ROOM_ while parsing svg map.");
         break;
   }
   
   int i = 0;
   
   for (cur_node = a_node; cur_node; cur_node = cur_node->next)
   {
      if (cur_node->type == XML_ELEMENT_NODE)
      {
         if (xmlStrEqual(cur_node->name, (const xmlChar *) "line")
             && xmlGetProp(cur_node, (const xmlChar *) "lineId") != NULL
             && atoi((const char *) xmlGetProp(cur_node, (const xmlChar *) "lineId")) >= start_line_id
             && atoi((const char *) xmlGetProp(cur_node, (const xmlChar *) "lineId")) <= end_line_id)
         {
            
            xmlChar *x1 = xmlGetProp(cur_node, (const xmlChar *) "x1");
            xmlChar *y1 = xmlGetProp(cur_node, (const xmlChar *) "y1");
            //xmlChar *x2 = xmlGetProp(cur_node, (const xmlChar *) "x2");
            //xmlChar *y2 = xmlGetProp(cur_node, (const xmlChar *) "y2");
            //xmlChar *lineId = xmlGetProp(cur_node, (const xmlChar *) "lineId");
            points[i].x = (double) atoi((const char *) x1);
            points[i].y = (double) atoi((const char *) y1);
            ++i;
            //printf("Line %i: x1 = %s y1 = %s x2 = %s y2 = %s\n", atoi((const char *) lineId), x1, y1, x2, y2);
         }
      }
   }
   return i;
}


int get_polygon_from_file(point *buffer, const char *filename, int room) {
   
   xmlDoc *doc = NULL;
   xmlNode *root_element = NULL;
   
   /*
    * this initialize the library and check potential ABI mismatches
    * between the version it was compiled for and the actual shared
    * library used.
    */
   LIBXML_TEST_VERSION
   
   /*parse the file and get the DOM */
   doc = xmlReadFile(filename, NULL, 0);
   
   if (doc == NULL) {
      mikes_log(ML_ERR, "SVG parse error: could not parse svg file");
   }
   
   /*Get the root element node */
   root_element = xmlDocGetRootElement(doc);
   
   int vert_n = parse_element(root_element->children, buffer, room);
   
   /* free the document */
   xmlFreeDoc(doc);
   /* Free the global variables that may have been allocated by the parser. */
   xmlCleanupParser();
   
   return vert_n;
}


//  public domain function by Darel Rex Finley, 2006



//  Determines the intersection point of the line defined by points A and B with the
//  line defined by points C and D.
//
//  Returns ABpos if the intersection point was found.
//  Returns -1/-2 if there is no determinable intersection point.
//  source: http://alienryderflex.com/intersect/

double line_intersection(
                         double Ax, double Ay,
                         double Bx, double By,
                         double Cx, double Cy,
                         double Dx, double Dy)
{
   
   double  distAB, theCos, theSin, newX, ABpos ;
   
   //  Fail if either line is undefined.
   if ((Ax==Bx && Ay==By) || (Cx==Dx && Cy==Dy)) return -1;
   
   //  (1) Translate the system so that point A is on the origin.
   Bx-=Ax; By-=Ay;
   Cx-=Ax; Cy-=Ay;
   Dx-=Ax; Dy-=Ay;
   
   //  Discover the length of segment A-B.
   distAB=sqrt(Bx*Bx+By*By);
   
   //  (2) Rotate the system so that point B is on the positive X axis.
   theCos=Bx/distAB;
   theSin=By/distAB;
   newX=Cx*theCos+Cy*theSin;
   Cy  =Cy*theCos-Cx*theSin; Cx=newX;
   newX=Dx*theCos+Dy*theSin;
   Dy  =Dy*theCos-Dx*theSin; Dx=newX;
   
   //  Fail if the lines are parallel.
   if (Cy==Dy) return -2;
   
   //  (3) Discover the position of the intersection point along line A-B.
   ABpos=Dx+(Cx-Dx)*Dy/(Dy-Cy);
   
//   //  (4) Apply the discovered position to line A-B in the original coordinate system.
//   *X=Ax+ABpos*theCos;
//   *Y=Ay+ABpos*theSin;
   
   //  Success.
   return ABpos;
}

double line_inter_poly_i(double Ax, double Ay, double Bx, double By, int i)
{
   return line_intersection(poly_i[i].x, poly_i[i].y,
                            poly_i[(i+1) % NUMBER_OF_VERTICES_I].x, poly_i[(i+1) % NUMBER_OF_VERTICES_I].y,
                            Ax, Ay,
                            Bx, By);
}

int init_mcl(){
   pthread_mutex_init(&lidar_mcl_lock, 0);
   
   // seed random generator
   time_t t;
   srand((unsigned) time(&t));
   
   // get vertices of polygons from file
   vert_n_i = get_polygon_from_file(poly_i, "mapa_pavilonu_I.svg", ROOM_I);
   vert_n_a = get_polygon_from_file(poly_a, "mapa_pavilonu_I.svg", ROOM_ATRIUM);
   vert_n_h = get_polygon_from_file(poly_h, "mapa_pavilonu_I.svg", ROOM_H3_H6);
   
   mikes_log_val(ML_INFO, "vert_n_i = ", vert_n_i);
   mikes_log_val(ML_INFO, "vert_n_a = ", vert_n_a);
   mikes_log_val(ML_INFO, "vert_n_h = ", vert_n_h);
   
   for (int i = 0; i < HYPO_COUNT; i++){
      
      double rand_x = 0;
      double rand_y = 0;
      
      do {
         rand_x = rand() % MAP_W;
         rand_y = rand() % MAP_H;
      } while (!is_in_corridor(rand_x, rand_y));
      
      
      hypo[0][i].x = hypo[1][i].x = rand_x;
      hypo[0][i].y = hypo[1][i].y = rand_y;
      hypo[0][i].alpha = hypo[1][i].alpha = rand() % 360;
      hypo[0][i].w = hypo[1][i].w = 0.3;
      
      //        mikes_log_val(ML_INFO, "hypo id: ", i);
      //        mikes_log_double2(ML_INFO, "hypo pos: ", hypo[0][i].x,hypo[0][i].y);
      //        mikes_log_double2(ML_INFO, "hypo v&a: ", hypo[0][i].w,hypo[0][i].alpha);
      
      
   }
   return 0;
}

void get_mcl_data(hypo_t *buffer)
{
   pthread_mutex_lock(&lidar_mcl_lock);
   memcpy(buffer, hypo[activeHypo], sizeof(hypo_t) * HYPO_COUNT);
   pthread_mutex_unlock(&lidar_mcl_lock);
}

double normAlpha(double alpha){
   if(alpha < 0){
      while(alpha < 0)
         alpha += 360;
   }
   else
      while(alpha >= 360)
         alpha -= 360;
   return alpha;
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

double get_min_intersection_dist(double x1, double y1, double x2, double y2) {
   
   double min_dist = MAP_H + MAP_W;
   double dist = 0;
   
   for (int i = 0; i < NUMBER_OF_VERTICES_I; ++i) {
      dist = line_intersection(x1, y1,
                               x2, y2,
                               poly_i[i].x, poly_i[i].y,
                               poly_i[(i+1) % NUMBER_OF_VERTICES_I].x, poly_i[(i+1) % NUMBER_OF_VERTICES_I].y);
      if (dist < min_dist) {
         min_dist = dist;
      }
   }
   for (int i = 0; i < NUMBER_OF_VERTICES_A; ++i) {
      dist = line_intersection(x1, y1,
                               x2, y2,
                               poly_a[i].x, poly_a[i].y,
                               poly_a[(i+1) % NUMBER_OF_VERTICES_A].x, poly_a[(i+1) % NUMBER_OF_VERTICES_A].y);
      if (dist < min_dist) {
         min_dist = dist;
      }
   }
   for (int i = 0; i < NUMBER_OF_VERTICES_H; ++i) {
      dist = line_intersection(x1, y1,
                               x2, y2,
                               poly_h[i].x, poly_h[i].y,
                               poly_h[(i+1) % NUMBER_OF_VERTICES_H].x, poly_h[(i+1) % NUMBER_OF_VERTICES_H].y);
      if (dist < min_dist) {
         min_dist = dist;
      }
   }
   return min_dist;
}

double getp( double x, double y){
   return fmax(-0.001*abs(x*x + 4*y*y) + 0.9, 0);
}

int mcl_update(double traveled, int heading, lidar_data_type liddata){
   mikes_log_double(ML_INFO, "MCL New data - traveled:", traveled);
   mikes_log_val(ML_INFO, "MCL New data - heading:", heading);
   
   pthread_mutex_lock(&lidar_mcl_lock);
   activeHypo = 1-activeHypo;
   
//   // ML_DEBUG
//   for(int i = 0; i < HYPO_COUNT; i++){
//      mikes_log_val(ML_DEBUG, "hypo id premove: ", i);
//      mikes_log_val2(ML_DEBUG, "hypo pos: ", hypo[1-activeHypo][i].x,hypo[1-activeHypo][i].y);
//      mikes_log_val2(ML_DEBUG, "hypo v&a: ", hypo[1-activeHypo][i].w*100,hypo[1-activeHypo][i].alpha);
//   }
   
   // poposuvame bodky podla pohybu a ratame pravdepodobnost
   for (int i = 0; i < HYPO_COUNT; i++) {
      
      double alpha_h = normAlpha(hypo[1-activeHypo][i].alpha+heading);
      hypo[1-activeHypo][i].x += traveled * cos(alpha_h * M_PI/180.0);
      hypo[1-activeHypo][i].y -= traveled * sin(alpha_h * M_PI/180.0);
      hypo[1-activeHypo][i].alpha = alpha_h;
      
      // ak bodka vysla z chodby, tak vahu bodke nastavime na nulu a dalej nepokracujeme
      if(!is_in_corridor(hypo[1-activeHypo][i].x, hypo[1-activeHypo][i].y)) {
         hypo[1-activeHypo][i].w = 0;
         continue;
      }
      
      
      //sensor position // 22 je vzdialenost senzora od stredu robota? v cm?
      double possx = hypo[1-activeHypo][i].x + cos(hypo[1-activeHypo][i].alpha*M_PI/180.0) * 22;
      double possy = hypo[1-activeHypo][i].y - sin(hypo[1-activeHypo][i].alpha*M_PI/180.0) * 22;
      //tag position
      //double minposx;
      //double minposy;
      
      
      // ratame pravdepodobnost ze sa robot vyskytuje na danej bodke
      /*
       porovname kazdy luc aky je namerany a aky by pre tuto danu bodku mal vyjst.
       dlzku nameraneho luca vieme, aj uhol
       vypocitame pre dany uhol kazdeho luca aka by dlzka mala vyjst
       */
      if (i == 1) {
         for (int j = 0; j < liddata.count; ++j) {
            uint16_t measured_distance = liddata.distance[j] / 4; // Actual distance = distance_q2 / 4 mm
            
            uint16_t angle_64 = liddata.angle[j];
            double angle = angle_64 / 64.0;
            double computed_distance_double = get_min_intersection_dist(possx, possy, possx + cos(angle*M_PI/180.0), possy - sin(angle*M_PI/180.0));
            uint16_t computed_distance = (uint16_t) (computed_distance_double * 10);
            //mikes_log_val2(ML_DEBUG, "measured | computed ", measured_distance, computed_distance);
         }
      }
   }
   
   
//   for(int i = 0; i< HYPO_COUNT; i++){
//      mikes_log_val(ML_DEBUG, "hypo id postmove: ", i);
//      mikes_log_val2(ML_DEBUG, "hypo pos: ", hypo[1-activeHypo][i].x,hypo[1-activeHypo][i].y);
//      mikes_log_val2(ML_DEBUG, "hypo v&a: ", hypo[1-activeHypo][i].w*100,hypo[1-activeHypo][i].alpha);
//   }
   
   // TODO
   // ???
   double cumP[HYPO_COUNT];
   double last = 0;
   for(int i = 0; i<= HYPO_COUNT; i++){
      last += hypo[1-activeHypo][i].w;
      cumP[i] = last;
   }
   
   // TODO
   // vygenerujeme nove bodky a zasumime ich
   int i;
   for(i = 0; i < HYPO_COUNT*0.9; i++){
      double next = (double)rand() / (double)RAND_MAX * last;
      for(int j = 0; j< HYPO_COUNT; j++){
         if( next <= cumP[j]){
            hypo[activeHypo][i].x = hypo[1-activeHypo][j].x + generateGaussianNoise(0, 0.03*traveled);
            hypo[activeHypo][i].y = hypo[1-activeHypo][j].y + generateGaussianNoise(0, 0.03*traveled);
            hypo[activeHypo][i].alpha = normAlpha(hypo[1-activeHypo][j].alpha + generateGaussianNoise(0, heading*0.05));
            hypo[activeHypo][i].w = hypo[1-activeHypo][j].w;
            break;
         }
      }
   }
   
   // generujeme novych nahodnych 10% bodiek
   for(; i< HYPO_COUNT; i++){
      
      double rand_x = 0;
      double rand_y = 0;
      
      do {
         rand_x = rand() % MAP_W;
         rand_y = rand() % MAP_H;
      } while (!is_in_corridor(rand_x, rand_y));
      
      hypo[0][i].x = hypo[1][i].x = rand_x;
      hypo[0][i].y = hypo[1][i].y = rand_y;
      hypo[0][i].alpha = hypo[1][i].alpha = rand() % 360;
      hypo[0][i].w = hypo[1][i].w = 0.01;
      
   }
   
   pthread_mutex_unlock(&lidar_mcl_lock);
   
   return 0;
}

