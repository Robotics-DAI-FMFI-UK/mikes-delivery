#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "lidar.h"
#include "base_module.h"
#include "mcl.h"
#include "pose.h"
#include "ride.h"

#define NORMAL_SPEED 10
#define AVOIDANCE_SPEED NORMAL_SPEED / 2
#define AVOIDANCE_DISTANCE_THRESHOLD 400

#define ANGLE_TOLERANCE_FOR_STRAIGHT_MOVEMENT 0.2

void rotate_robot(int angle, base_data_type *old_data)
{
	base_data_type base_d;
	//printf("steering: quarter perimeter = %.1f\n", ((WHEELS_DISTANCE * M_PI) / 360.0) * (double)abs(angle));
	int new_counter = MM2COUNTER(((WHEELS_DISTANCE * M_PI) / 360.0) * (double)abs(angle));
	
	if (angle > 0)
		set_motor_speeds(NORMAL_SPEED, -NORMAL_SPEED);
	else
		set_motor_speeds(-NORMAL_SPEED, NORMAL_SPEED);
	
	do
	{
		usleep(100000);
		get_base_data(&base_d);
	} while (abs(old_data->counterA - base_d.counterA) > new_counter);
}

void *steering_thread(void *arg)
{
	base_data_type base_data;
	lidar_data_type lidar_data;
	pose_type pose;
	
	sleep(3);
	reset_counters();
	
	get_base_data(&base_data);
	get_lidar_data(&lidar_data);
	
	set_motor_speeds(NORMAL_SPEED, NORMAL_SPEED);
	
	long left_sum;
	int left_count;
	long right_sum;
	int right_count;
	double left_avg;
	double right_avg;
	int left_min_avg;
	int right_min_avg;
	int front_min_avg;
	
	//rotate_robot(90, &base_data);
	//rotate_robot(-90, &base_data);
	
	
	while (program_runs)
	{
		usleep(100000);
		get_base_data(&base_data);
		get_lidar_data(&lidar_data);
		get_pose(&pose);
		
		//printf("steer: counterA %ld counterB %ld, heading %d, pose [%.2f, %.2f]\n", base_data.counterA, base_data.counterB, base_data.heading, pose.x, pose.y);
		
		// watching obstacles
		// left  (240, 315)
		// front <315, 45>
		// right (45, 120)
		
		left_sum = 0;
		left_count = 0;
		left_avg = 0;
		left_min_avg = 6000;
		right_sum = 0;
		right_count = 0;
		right_avg = 0;
		right_min_avg = 6000;
		front_min_avg = 6000;
		
		int ppl = 0;
		int ppr = 0;
		int ppf = 0;
		
		int jl = 0;
		int jr = 0;
		int jf = 0;
		for (int i = 0; i < lidar_data.count; i++)
		{
			if (lidar_data.distance[i] == 0)
				continue;
			
			double angle = lidar_data.angle[i] / 64;
			int distance = lidar_data.distance[i] / 4;
			
			
			if (angle > 240 && angle < 315)
			{ // left
				left_count++;
				left_sum += distance;
				ppl += distance;
				jl++;
			}
			else if ((angle > 315 && angle < 360) || (angle >= 0 && angle < 45))
			{ // front
				ppf += distance;
				jf++;
			}
			else if  (angle > 45 && angle < 120)
			{ // right
				right_count++;
				right_sum += distance;
				ppr += distance;
				jr++;
			}
			if (jl == 3)
			{
				ppl /= 3;
				if (ppl < left_min_avg)
					left_min_avg = ppl;
				ppl = 0;
				jl = 0;
			}
			if (jr == 3)
			{
				ppr /= 3;
				if (ppr < right_min_avg)
					right_min_avg = ppr;
				ppr = 0;
				jr = 0;
			}
			if (jf == 3)
			{
				ppf /= 3;
				if (ppf < front_min_avg)
					front_min_avg = ppf;
				ppf = 0;
				jf = 0;
			}
			
		}
		right_avg = right_sum / (double) (right_count);
		left_avg = left_sum / (double) (left_count);
		
		//printf("steering: min)avgs l r f: %d %d %d\n", left_min_avg, right_min_avg, front_min_avg);
		
		if (front_min_avg < AVOIDANCE_DISTANCE_THRESHOLD)
		{
			set_motor_speeds(0,0);
			if (left_avg > right_avg)
				rotate_robot(-31, &base_data);
			else
				rotate_robot(31, &base_data);
			//printf("steering: obstacle in the front, robot rotated\n");
			continue;
		} 
		else if (left_min_avg < AVOIDANCE_DISTANCE_THRESHOLD)
		{
			set_motor_speeds(NORMAL_SPEED, AVOIDANCE_SPEED);
			//printf("steering: obstacle on left side: %d %d\n", NORMAL_SPEED, AVOIDANCE_SPEED);
			continue;
		}
		else if (right_min_avg < AVOIDANCE_DISTANCE_THRESHOLD)
		{
			set_motor_speeds(AVOIDANCE_SPEED, NORMAL_SPEED);
			//printf("steering: obstacle on right side: %d %d\n", AVOIDANCE_SPEED, NORMAL_SPEED);
			continue;
		} 
		
		
		
		//maybe some if?
		{
			double desired_heading = get_desired_relative_heading(pose);
			//printf("steering: desired_heading %.1f\n", desired_heading*180 / M_PI);
			int desired_heading_in_degrees = (int)(desired_heading * 180.0 / M_PI);
			
			if (abs(desired_heading_in_degrees) > 90)
			{	//stop and make rotation
				rotate_robot(desired_heading_in_degrees / 6, &base_data);
			}
			else
			{
				int slower_velocity = NORMAL_SPEED * (90 - abs(desired_heading_in_degrees)) / (double)90;
                                printf("--------------[%f,%f], %f = %f --------------%d\n", pose.x, pose.y, pose.heading, pose.heading / M_PI * 180.0, desired_heading_in_degrees);
				//	printf("steering: normal speed subtract = %f\n", fabs(desired_heading*180 / M_PI) / NORMAL_SPEED);
				if (desired_heading < -ANGLE_TOLERANCE_FOR_STRAIGHT_MOVEMENT)
				{ //turning left
					set_motor_speeds(slower_velocity, NORMAL_SPEED);
					printf("steering: turning left:  %d %d\n", slower_velocity, NORMAL_SPEED);
				}
				else if (desired_heading > ANGLE_TOLERANCE_FOR_STRAIGHT_MOVEMENT)
				{ //turning right
					set_motor_speeds(NORMAL_SPEED, slower_velocity);
					printf("steering: turning right: %d %d\n", NORMAL_SPEED, slower_velocity);
				}
				else 
				{ // straight motion
					set_motor_speeds(NORMAL_SPEED, NORMAL_SPEED);
					printf("steering: going forward: %d %d\n", NORMAL_SPEED, NORMAL_SPEED);
				}
			}
			
		}
	}
	mikes_log(ML_INFO, "steering quits.");
	threads_running_add(-1);
	return 0;
}

void init_steering()
{
	pthread_t t;
	if (pthread_create(&t, 0, steering_thread, 0) != 0)
	{
		perror("mikes:steering");
		mikes_log(ML_ERR, "creating steering thread");
	}
	else threads_running_add(1);
}
