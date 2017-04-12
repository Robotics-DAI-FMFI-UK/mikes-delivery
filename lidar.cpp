#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#include <rplidar.h>

extern "C" {
	
#include "mikes.h"
#include "lidar.h"
#include "mikes_logs.h"

}

#define LIDAR_PORT "/dev/lidar"
#define LIDAR_BAUD_RATE 115200

using namespace rp::standalone::rplidar;

static pthread_mutex_t lidar_lock;
static rplidar_response_measurement_node_t *lidar_data;
static size_t lidar_data_count;

static RPlidarDriver *drv;

static rplidar_response_measurement_node_t *local_data;
static size_t local_data_count;

void connect_lidar()
{
    // create the driver instance
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
		mikes_log(ML_ERR, "insufficent memory, exit");
        return;
    }

    rplidar_response_device_health_t healthinfo;
    rplidar_response_device_info_t devinfo;
    do {
        // try to connect
        if (IS_FAIL(drv->connect(LIDAR_PORT, LIDAR_BAUD_RATE))) {
			mikes_log(ML_ERR, "Error, cannot bind to the specified serial port");
            break;
        }

        // retrieving the device info
        ////////////////////////////////////////
        u_result op_result = drv->getDeviceInfo(devinfo);

        if (IS_FAIL(op_result)) {
            if (op_result == RESULT_OPERATION_TIMEOUT) {
                // you can check the detailed failure reason
                mikes_log(ML_ERR, "Error, operation time out");
            } else {
				mikes_log_val(ML_ERR, "Error, unexpected error ", op_result);
                // other unexpected result
            }
            break;
        }

        // check the device health
        ////////////////////////////////////////
        op_result = drv->getHealth(healthinfo);
        if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            switch (healthinfo.status) {
            case RPLIDAR_STATUS_OK:
                mikes_log(ML_INFO, "RPLidar health status : OK.");
                break;
            case RPLIDAR_STATUS_WARNING:
                mikes_log(ML_INFO, "RPLidar health status : Warning.");
				mikes_log_val(ML_INFO, "lidar errorcode: ", healthinfo.error_code);
                break;
            case RPLIDAR_STATUS_ERROR:
                mikes_log(ML_INFO, "RPLidar health status : Error.");
				mikes_log_val(ML_INFO, "lidar errorcode: ", healthinfo.error_code);
                break;
            }

        } else {
            mikes_log_val(ML_ERR, "Error, cannot retrieve the lidar health code: ", op_result);
            break;
        }


        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            mikes_log(ML_ERR, "Error, rplidar internal error detected. Please reboot the device to retry.");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            break;
        }

        drv->startMotor();

    } while(0);
	
    mikes_log(ML_INFO, "lidar connected");
}

void *lidar_thread(void *args)
{
	while (program_runs)
    {
		int erri = 0;
		while (erri < 30) {
			if (IS_FAIL(drv->startScan())) 
			{
				mikes_log_val(ML_ERR, "Error, cannot start the scan operation. Trying again.", erri);
				++erri;
				usleep(50000);
			} else {
				break;
			}
		}
		if (erri == 30) { 
			mikes_log(ML_ERR, "Error, cannot start the scan operation. End.");
			break;
		}

		u_result ans;    
		local_data_count = MAX_LIDAR_DATA_COUNT;

		// fetech extactly one 0-360 degrees' scan
		ans = drv->grabScanData(local_data, local_data_count);
		if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
			drv->ascendScanData(local_data, local_data_count);

		} else {
        mikes_log_val(ML_ERR, "lidar error code: ", ans);
		}
		
        pthread_mutex_lock(&lidar_lock);
        memcpy(lidar_data, local_data, local_data_count * sizeof(rplidar_response_measurement_node_t));
        lidar_data_count = local_data_count;
        pthread_mutex_unlock(&lidar_lock);
        usleep(45000);
    }

    drv->stop();
    drv->stopMotor();

    RPlidarDriver::DisposeDriver(drv);

    free(lidar_data);
    free(local_data);
    
    mikes_log(ML_INFO, "lidar quits.");
    threads_running_add(-1);
    return 0;
}

void init_lidar()
{
    pthread_t t;
    lidar_data = (rplidar_response_measurement_node_t *) malloc(sizeof(rplidar_response_measurement_node_t) * MAX_LIDAR_DATA_COUNT);
    local_data = (rplidar_response_measurement_node_t *) malloc(sizeof(rplidar_response_measurement_node_t) * MAX_LIDAR_DATA_COUNT);
    if ((lidar_data == 0) || (local_data == 0) )
    {
      perror("mikes:lidar");
      mikes_log(ML_ERR, "insufficient memory");
      exit(1);
    }
    connect_lidar();
    pthread_mutex_init(&lidar_lock, 0);
    if (pthread_create(&t, 0, lidar_thread, 0) != 0)
    {
      perror("mikes:lidar");
      mikes_log(ML_ERR, "creating thread for lidar");
    }
    else threads_running_add(1);
}

void get_lidar_data(lidar_data_type *buffer)
{
    pthread_mutex_lock(&lidar_lock);
      for (size_t i = 0; i < lidar_data_count; ++i) {
            buffer->quality[i] = lidar_data[i].sync_quality >> 2;   // syncbit:1;syncbit_inverse:1;quality:6;
            buffer->angle[i] = lidar_data[i].angle_q6_checkbit >> 1; // check_bit:1;angle_q6:15;
            buffer->distance[i] = lidar_data[i].distance_q2;
      }
      buffer->count = lidar_data_count;
    pthread_mutex_unlock(&lidar_lock);
}

