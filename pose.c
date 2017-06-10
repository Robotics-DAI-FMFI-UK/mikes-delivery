#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>

#include "pose.h"
#include "base_module.h"
#include "mikes_logs.h"
#include "util.h"

static double pos_x, pos_y, pos_heading;
static long old_A = 0, old_B = 0;

static int pos_update_threshold_dist = 8;
static double straight_movement_threshold = 0.07;

static int wheels_distance = WHEELS_DISTANCE; // in mm
static int reverse_y = 0;
static int max_y = 0;

static pthread_mutex_t pose_module_lock;

void init_pose(int init_reverse_y, int init_max_y)
{
	pthread_mutex_init(&pose_module_lock, 0);
	pos_x = 0; pos_y = 0; pos_heading = 0;
	reverse_y = init_reverse_y;
	max_y = init_max_y;
}


void update_pose(base_data_type *base_data)
{
	double new_x, new_y, new_heading;

    double dA = COUNTER2MM((double)base_data->counterA - old_A) / 10.0;
	double dB = COUNTER2MM((double)base_data->counterB - old_B) / 10.0;
	

	
	if ((fabs(dA) + fabs(dB) < pos_update_threshold_dist))
	   return;

	FILE *f = fopen("poses.txt", "a+");
    fprintf(f, "%f %f %f\t", pos_x, pos_y, pos_heading);
	fprintf(f, "%.2f %.2f\t", dA, dB);
	
    old_A = base_data->counterA;
	old_B = base_data->counterB;

	double d = (dA + dB) / 2.0;
	double turning_ratio = 2;
	if ((fabs(dA) >= 1.0) || (fabs(dB) >= 1.0))
		turning_ratio = dA / (double)dB;
    //printf("turning_r = %.3f (pos=[%.2f, %.2f, %.2f], d=%.3f), ?=%.5f\n", turning_ratio, pos_x, pos_y, pos_heading, d, fabs(turning_ratio - 1.0));
    if (fabs(turning_ratio - 1.0) < straight_movement_threshold)
    {  // straight line movement
		new_x = pos_x + d * sin(pos_heading);
		new_y = pos_y + d * cos(pos_heading);
		new_heading = pos_heading; 
		//printf("straight, new=[%.2f, %.2f, %.3f]\n", new_x, new_y, new_heading);
		fprintf(f,"s\t");
        }
	else if ((dA * dB < 0) && (fabs(fabs(dA) - fabs(dB)) < 0.3))
	{  // rotating around robot center
		new_x = pos_x;
		new_y = pos_y;
		new_heading = pos_heading + dA / (2.0 * wheels_distance / 10.0);		
		//printf("rotate, new=[%.2f, %.2f, %.3f]\n", new_x, new_y, new_heading);
		fprintf(f,"c\t");
	}
	else // moving both wheels forward along circular trajectory
	{
                int center_on_the_right = 1;
                double r1;
                if (fabs(dB) > fabs(dA))
                {
                    center_on_the_right = -1;
		    r1 = wheels_distance * dA / (dB - dA) / 10.0;
		    fprintf(f,"l\t");
                }
                else
                {
		    r1 = wheels_distance * dB / (dA - dB) / 10.0;
		    fprintf(f,"r\t");
                }
		double r = (r1 + wheels_distance / 20.0);
		double beta = d / r;
		double Cx = pos_x + r * sin(pos_heading + center_on_the_right * M_PI / 2.0);
		double Cy = pos_y + r * cos(pos_heading + center_on_the_right * M_PI / 2.0);
		new_x = Cx + r * sin(pos_heading - center_on_the_right * M_PI / 2.0 + center_on_the_right * beta);
		new_y = Cy + r * cos(pos_heading - center_on_the_right * M_PI / 2.0 + center_on_the_right * beta);
		new_heading = pos_heading + center_on_the_right * beta;
		fprintf(f, " <r1=%.3f, beta=%.3f, c=[%.3f,%.3f]>\t", r1, beta, Cx, Cy);
		//printf("circle, new=[%.2f, %.2f, %.3f]\n", new_x, new_y, new_heading);
	}
	
    pthread_mutex_lock(&pose_module_lock);
	    pos_x = new_x;
		pos_y = new_y;
	    pos_heading = rad_normAlpha(new_heading);
    pthread_mutex_unlock(&pose_module_lock);
    fprintf(f, "%f %f %f\n", pos_x, pos_y, pos_heading);
    fclose(f);
}

void get_pose(pose_type *pose)
{
    pthread_mutex_lock(&pose_module_lock);

		pose->x = pos_x;
		if (reverse_y)
			pose->y = max_y - pos_y;
		else
			pose->y = pos_y;
		pose->heading = pos_heading;
                pose->row = (int)(pose->y / STATE_WIDTH);
                pose->col = (int)(pos_x / STATE_WIDTH);

    pthread_mutex_unlock(&pose_module_lock);
}

void set_pose(double newx, double newy, double newheading)
{
	base_data_type reset_base_data;
	get_base_data(&reset_base_data);
	
    pthread_mutex_lock(&pose_module_lock);

		pos_x = newx;
		if (reverse_y)
			pos_y = max_y - newy;
		else
			pos_y = newy;
		pos_heading = newheading;
		old_A = reset_base_data.counterA;
		old_B = reset_base_data.counterB;
		
    pthread_mutex_unlock(&pose_module_lock);

    mikes_log_double2(ML_DEBUG, "pose: set pose x y: ", pos_x, pos_y);
    mikes_log_double(ML_DEBUG, "pose: set pose heading: ", pos_heading);
    mikes_log_double2(ML_DEBUG, "pose: set pose old_A old_B: ", old_A, old_B);
}
