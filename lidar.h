#ifndef _LIDAR_H
#define _LIDAR_H

#include <pthread.h>
#include <math.h>
#include <rplidar.h>
#define MAX_LIDAR_DATA_COUNT 720

extern pthread_mutex_t lidar_lock;
extern rplidar_response_measurement_node_t *lidar_data;

void init_lidar();
int get_lidar_data(int *buffer);

/* ray: 0..range_data_count - 1, returns: 0-360 */
int lidarray2azimuth(int ray);

/* alpha: -180..360, returns: ray 0..max, max = range_data_count - 1 (out of range clips to 0 or to max) */
int azimuth2lidarray(int alpha);

#endif
