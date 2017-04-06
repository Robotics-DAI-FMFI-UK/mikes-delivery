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


void *navigation_thread(void *arg)
{
    base_data_type base_data;
    sleep(3);

    rplidar_response_measurement_node_t lidar_nav_data;
//    size_t lidar_nav_data_count;
    get_base_data(&base_data);
    original_heading = base_data.heading;
    mikes_log_val(ML_INFO, "original heading: ", original_heading);

    reset_counters();
    
    //debug_navigation();

    set_motor_speeds(10,10);
    follow_azimuth(original_heading);
    mikes_log(ML_INFO, "navigate: put");
    //while (!start_automatically) 
    //  usleep(10000);
    int ii = 0;
    while (program_runs)
    {
      get_base_data(&base_data);
      if (ii == 30) stop_now();
//      lidar_nav_data_count = get_lidar_data(&lidar_nav_data);
//      printf("sum of beams: %d\n", lidar_nav_data_count);
      usleep(100000);
      ++ii;
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
