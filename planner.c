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
#include "astar.h"
#include "base_module.h"
#include "pose.h"
#include "mcl.h"

static int endx = 180;
static int endy = 47;

int planner_trigger;

int world[STATES_H * STATES_W];


int WorldAt(int col, int row)
{
	if (row >= 0 && row < STATES_H && col >= 0 && col < STATES_W) {
		return world[row*STATES_W + col];
	} else {
		return -1;
	}
}

int init_world()
{
   // iniciovanie grid mapy => ak stvorcek je pretaty nejakou stenou, tak ho oznacime 0 (vykreslene cervenou) 
   for (int row = 0; row < STATES_H; ++row) {
      for (int col = 0; col < STATES_W; ++col) {

         int wrow = row*STATE_WIDTH;
         int wcol = col*STATE_WIDTH;
         if (get_line_intersection(wcol, wrow, wcol + STATE_WIDTH, wrow) > 0
             || get_line_intersection(wcol + STATE_WIDTH, wrow, wcol + STATE_WIDTH, wrow + STATE_WIDTH) > 0
             || get_line_intersection(wcol + STATE_WIDTH, wrow + STATE_WIDTH, wcol, wrow + STATE_WIDTH) > 0
             || get_line_intersection(wcol, wrow + STATE_WIDTH, wcol, wrow) > 0)
         {
            world[row*STATES_W + col] = 1;
         } else {
            world[row*STATES_W + col] = 0;
         }
         
      }
   }
   
   for (int row = 1; row < STATES_H-1; ++row) {
      for (int col = 1; col < STATES_W-1; ++col) {
		  if (world[row*STATES_W + col] == 0 && 
		      (world[(row-1)*STATES_W + col] == 1
		      || world[(row+1)*STATES_W + col] == 1
		      || world[(row)*STATES_W + col-1] == 1
		      || world[(row)*STATES_W + col+1] == 1
		      || world[(row-1)*STATES_W + col-1] == 1
		      || world[(row-1)*STATES_W + col+1] == 1
		      || world[(row+1)*STATES_W + col-1] == 1
		      || world[(row+1)*STATES_W + col+1] == 1))
		  {
		      world[(row)*STATES_W + col] = 2;
		  }
	  }
   }
   for (int row = 1; row < STATES_H-1; ++row) {
      for (int col = 1; col < STATES_W-1; ++col) {
		  if (world[row*STATES_W + col] == 2)
		      world[row*STATES_W + col] = 1;
	  }
   }

	//// write world to console
	//for (int row = 0; row < STATES_H; ++row) {
		//printf("%.3d ", row);
		//for (int col = 0; col < STATES_W; ++col) {
			//if (col % 20 == 0)
				//printf("|");
			//printf("%d", WorldAt(col, row));
		//}
		//printf("\n"); 
	//}
	
	
	return 1;
}

void *planner_thread(void *arg)
{
	while (program_runs)
	{
                if (planner_trigger)
                {
                    planner_trigger = 0;
                    mikes_log(ML_INFO, "planger trigger");
		
                    pose_type pose;
                    get_pose(&pose);
                    // printf("pose col %d, row %d\n", pose.col, pose.row);
                    mikes_log_val2(ML_DEBUG, "pose col row", pose.col, pose.row);

  		    int astar_result = astar(world, STATES_W, STATES_H, pose.col, pose.row, endx, endy, rectilinear);
  		    mikes_log_val(ML_INFO, "astar result:", astar_result);
                }
		sleep(1);
	}
	
	mikes_log(ML_INFO, "planner quits.");
	threads_running_add(-1);
	return 0;
}

int init_planner()
{
   planner_trigger = 0;
   pthread_t t;
   if (pthread_create(&t, 0, planner_thread, 0) != 0)
   {
      perror("mikes:planner");
      mikes_log(ML_ERR, "creating planner thread");
   }
   else threads_running_add(1);

  init_world();

  int startx = mikes_config.start_x;
  int starty = mikes_config.start_y;

  int endx = 180;
  int endy = 47;

  int astar_result = astar(world, STATES_W, STATES_H, startx, starty, endx, endy, rectilinear);
  mikes_log_val(ML_INFO, "astar result:", astar_result);

  set_pose((startx*STATE_WIDTH + STATE_WIDTH / 2.0), (starty*STATE_WIDTH + STATE_WIDTH / 2.0), M_PI);
  pose_type mypose;
  get_pose(&mypose);
  printf("pose x %f.1 y %f.1 col %d row %d\n", mypose.x, mypose.y, mypose.col, mypose.row);

   return 0;
}
