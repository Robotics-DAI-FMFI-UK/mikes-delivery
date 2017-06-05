#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "mikes.h"
#include "ride.h"
#include "astar.h"
#include "mikes_logs.h"
#include "mcl.h"
#include "planner.h"

static int min(int a, int b) { return (a > b)?b:a; }

static void get_desired_target_position_in_cm(pose_type pose, int *goal_x, int *goal_y)
{
    double min_dist2 = MAP_H + MAP_W;
    min_dist2 *= min_dist2;

    path_lock();

        int detected_path_step = -1;
	
	for (int i = 0; i < path_len; i++)
	{
		double vect_x = fabs(pose.x - (path[i][1]*STATE_WIDTH + (STATE_WIDTH / 2.0)));
		double vect_y = fabs(pose.y - (path[i][0]*STATE_WIDTH + (STATE_WIDTH / 2.0)));
		double dist2 = vect_x*vect_x + vect_y*vect_y;
		
		if (dist2 < min_dist2)
		{
			min_dist2 = dist2;
                        detected_path_step = i;
		}
	}

	//printf("ride: actual square (%d, %d) of pose[%.2f %.2f]\n", sqx, sqy, pose.x, pose.y);
	//mikes_log_val2(ML_DEBUG, "ride: pose.x pose.y", pose.x, pose.y);

        int desired_path_step = min(path_len - 1, detected_path_step + 3);

        if (min_dist2 > LOST_PATH_MCL_TRIGGER_DISTANCE) planner_trigger = 1;

	//mikes_log_val(ML_DEBUG, "ride: desired sq", desired_sq)u
	
	*goal_x = (path[desired_path_step][1] * STATE_WIDTH + STATE_WIDTH / 2.0); 
	*goal_y = (path[desired_path_step][0] * STATE_WIDTH + STATE_WIDTH / 2.0);
	mikes_log_val2(ML_DEBUG, "new goal:", *goal_x, *goal_y);
	
    path_unlock();
}

double get_desired_relative_heading(pose_type pose)
{
	//printf("ride: pose[%.1f, %.1f] goal[%.1f, %.1f]\n", pose.x, pose.y, goal_x, goal_y);

        int goal_x, goal_y;

        get_desired_target_position_in_cm(pose, &goal_x, &goal_y);
	
	double vector_x = goal_x - pose.x;
	double vector_y = goal_y - pose.y;
	//double vector_len = sqrt(vector_x*vector_x + vector_y*vector_y);
	
	//printf("ride: vector(%.1f, %.1f) len = %.1f\n", vector_x, vector_y, vector_len);
	
	//double robot_x = sin(pose.heading);
	//double robot_y = -cos(pose.heading);
	//double robot_len = sqrt(robot_x*robot_x + robot_y*robot_y);

	//printf("ride: robot(%.1f, %.1f) len = %.1f\n", robot_x, robot_y, robot_len);
	
	double heading_s = angle_rad_difference(pose.heading, M_PI / 2.0 - atan2(vector_y, vector_x)); //signed heading
	
	//printf("ride: heading %.1f\n", heading_s*180/M_PI);
	
	
	return heading_s;
}

