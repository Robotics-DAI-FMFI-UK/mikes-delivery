#ifndef _TIM571_H_
#define _TIM571_H_

#include <pthread.h>
#include <math.h>

#define TIM571_DATA_COUNT 811
#define TIM571_MAX_DATA_COUNT TIM571_DATA_COUNT
#define TIM571_TOTAL_ANGLE (3 * M_PI / 2)
#define TIM571_TOTAL_ANGLE_DEG 270
#define TIM571_SIZE_OF_ONE_STEP (TIM571_TOTAL_ANGLE / (TIM571_DATA_COUNT - 1))
#define TIM571_SIZE_OF_ONE_DEG ((TIM571_DATA_COUNT - 1) / TIM571_TOTAL_ANGLE_DEG)
#define TIM571_PORT 2111
#define TIM571_ADDR "169.254.0.5"
#define TIM571_MAX_DISTANCE 15000


// status information received from the sensor (typical values in comments)
typedef struct tim571_status_struct {
	uint16_t firmware_version;     // 1
	uint16_t sopas_device_id;      // configurable in SOPAS
    uint32_t serial_number;        
    uint8_t error;                 // 0 = device is ready
    uint32_t scanning_frequency;   // 1500  in 1/100 Hz
    float multiplier;              // 1
    int32_t starting_angle;        // -450000 (in 1/10000 of degree)
    uint32_t angular_step;         // 3333    (in 1/10000 of degree)
    uint16_t data_count;           // number of distance/rssi data
    uint8_t rssi_available;        // 0 = not receiving rssi data
    char name[17];                 // name configurable in SOPAS or "\0"
} tim571_status_data;


extern pthread_mutex_t tim571_lock;

void init_tim571();

void get_tim571_dist_data(uint16_t *buffer); // fills in data_count measured data values
void get_tim571_rssi_data(uint8_t *buffer);  // fills in data_count measured rssi values
void get_tim571_status_data(tim571_status_data *status_data); // fills in status data
void pretty_print_status_data(char *buffer, tim571_status_data *sd); // prints status data to string buffer (must have enough space!)

void test_tim571();

/* ray: 0..TIM571_DATA_COUNT - 1, returns: 0-360 */
int tim571_ray2azimuth(int ray);

/* alpha: -180..360, returns: ray 0..max, max = TIM571_DATA_COUNT - 1 (out of range clips to 0 or to max) */
int tim571_azimuth2ray(int alpha);

#endif
