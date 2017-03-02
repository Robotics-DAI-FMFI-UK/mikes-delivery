#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "range_sensor.h"
#include "base_module.h"
#include "util.h"

#define NORMAL_NAVIGATION_SPEED          12

static short original_heading;

void *navigation_thread(void *arg)
{
    base_data_type base_data;
    sleep(3);
    get_base_data(&base_data);
    original_heading = base_data.heading;
    mikes_log_val(ML_INFO, "original heading: ", original_heading);

    static segments_type segments;
    reset_counters();

    while (!start_automatically) 
      usleep(10000);

    while (program_runs)
    {
        get_base_data(&base_data);
        get_range_segments(&segments, 180*4, 145, 350);
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
