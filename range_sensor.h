#ifndef _RANGE_SENSOR_H
#define _RANGE_SENSOR_H

#include <pthread.h>
#include <math.h>

/* ray integer calculations are based on the fact that UST-10LX takes exactly 4 rays per degree */
#define RANGE_DATA_COUNT 1081
#define TOTAL_ANGLE (3 * M_PI / 2)
#define TOTAL_ANGLE_DEG 270
#define SIZE_OF_ONE_STEP (TOTAL_ANGLE / (RANGE_DATA_COUNT - 1))
#define SIZE_OF_ONE_DEG ((RANGE_DATA_COUNT - 1) / TOTAL_ANGLE_DEG)
#define HOKUYO_PORT 10940
#define HOKUYO_ADDR "169.254.0.10"
#define MAX_DISTANCE 8000

extern pthread_mutex_t range_sensor_lock;
extern int *range_data;

#define MAX_SEGMENTS (RANGE_DATA_COUNT / 3 + 1)

typedef struct seg_struc {
    int nsegs_found;           // number of segments found
    int dist[MAX_SEGMENTS];    // approx. distance to each segment in mm
    int width[MAX_SEGMENTS];   // approx. width of each segment in mm
    int alpha[MAX_SEGMENTS];   // direction to each segment centre [-135..135]
    int firstray[MAX_SEGMENTS];
    int lastray[MAX_SEGMENTS];
} segments_type;

void init_range_sensor();
void get_range_data(int* buffer);
void get_range_segments(segments_type *segments, int angular_detecting_range, int min_seg_size, int max_seg_size);

/* ray: 0..RANGE_DATA_COUNT - 1, returns: 0-360 */
int ray2azimuth(int ray);

/* alpha: -180..360, returns: ray 0..max, max = RANGE_DATA_COUNT - 1 (out of range clips to 0 or to max) */
int azimuth2ray(int alpha);

#endif
