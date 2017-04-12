#ifndef _LIDAR_H
#define _LIDAR_H

#include <stdint.h>
#include <pthread.h>

#define MAX_LIDAR_DATA_COUNT 720

typedef struct lidarstruct {
  uint16_t count;
  uint8_t quality[MAX_LIDAR_DATA_COUNT];
  uint16_t distance[MAX_LIDAR_DATA_COUNT];
  uint16_t angle[MAX_LIDAR_DATA_COUNT];
} lidar_data_type;

void init_lidar();
void get_lidar_data(lidar_data_type *buffer);

#endif
