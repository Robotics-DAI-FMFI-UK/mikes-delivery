#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "lidar.h"
#include "base_module.h"
#include "util.h"

#define NORMAL_NAVIGATION_SPEED          12
#define MAX_DATA_DISTANCE 6000
#define AVOIDANCE_DISTANCE_THRESHOLD 100000

typedef int bool;
#define true 1
#define false 0

static short original_heading;

void debug_navigation()
{
    while(1)
    {
        set_motor_speeds(30, 30);
        sleep(2);
        set_motor_speeds(-30, -30);
        sleep(2);
        stop_now();
        sleep(2);
    }
}

long get_sum_of_beams(lidar_data_type data)
{
  long sum_of_distance = 0;
  for (int i = 0; i < data.count; ++i)
  {
    if (data.distance[i] == 0)
      sum_of_distance += MAX_DATA_DISTANCE;
    else
      sum_of_distance += data.distance[i];
  }
  return sum_of_distance;
}

bool obstacle_in_data(lidar_data_type old_data, lidar_data_type new_data)
{
  int old_sum = get_sum_of_beams(old_data);
  int new_sum = get_sum_of_beams(new_data);
  if (old_sum - new_sum > AVOIDANCE_DISTANCE_THRESHOLD)
  {
    //stop now
    stop_now();
    mikes_log(ML_INFO, "Mikes sees obstacle.");
    return true;
  }
  return false;
}
bool no_obstacle_in_data(lidar_data_type old_data, lidar_data_type new_data)
{
  int old_sum = get_sum_of_beams(old_data);
  int new_sum = get_sum_of_beams(new_data);
  if (new_sum - old_sum > AVOIDANCE_DISTANCE_THRESHOLD)
  {
    //start moving
    set_motor_speeds(10,10);
    follow_azimuth(original_heading);
    mikes_log(ML_INFO, "Mikes sees no obstacle.");
    return false;
  }
  return true;
}


void *navigation_thread(void *arg)
{
    base_data_type base_data;
    lidar_data_type lidar_data;
    sleep(3);

    get_base_data(&base_data);
    original_heading = base_data.heading;
    mikes_log_val(ML_INFO, "original heading: ", original_heading);

    reset_counters();
    get_lidar_data(&lidar_data);

//    debug_navigation();

    set_motor_speeds(10,10);
    follow_azimuth(original_heading);
    mikes_log(ML_INFO, "navigate: put");
//    while (!start_automatically) 
//      usleep(10000);
    bool stopped = false;
    while (program_runs)
    {
      get_base_data(&base_data);

      lidar_data_type old_data = lidar_data;
      get_lidar_data(&lidar_data);
      printf("%ld\n",get_sum_of_beams(lidar_data));
      
      if (stopped == 1) {
        stopped = no_obstacle_in_data(old_data, lidar_data);
      } else {
        stopped = obstacle_in_data(old_data, lidar_data);
      }
      
      usleep(100000);
    }

    mikes_log(ML_INFO, "navigation quits.");
    threads_running_add(-1);
    return 0;
}

void init_navigation()
{
    pthread_t t;
    if (pthread_create(&t, 0, navigation_thread, 0) != 0)
    {
        perror("mikes:navigation");
        mikes_log(ML_ERR, "creating navigation thread");
    }
    else threads_running_add(1);
}
