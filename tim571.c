#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

#include "mikes.h"
#include "tim571.h"
#include "mikes_logs.h"

#define MAX_SENTENCE_LENGTH 10000

#define STX 2
#define ETX 3

pthread_mutex_t tim571_lock;

static uint16_t *local_data;
static uint8_t *local_rssi;

static int sockfd;

void connect_tim571()
{
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
    mikes_log(ML_ERR, "cannot open tim571 socket");
    perror("mikes:tim571");
        return;
    }

    struct sockaddr_in remoteaddr;
    remoteaddr.sin_family = AF_INET;
    remoteaddr.sin_addr.s_addr = inet_addr(TIM571_ADDR);
    remoteaddr.sin_port = htons(TIM571_PORT);

    if (connect(sockfd, (struct sockaddr*)&remoteaddr, sizeof(remoteaddr)) < 0)
    {
        mikes_log(ML_ERR, "connecting tim571 socket");
        perror("mikes:tim571");
        return;
    }

    mikes_log(ML_INFO, "tim571 connected");
}

void send_start_measurement()
{
    static const char *start_measurement = "\002sEN LMDscandata 1\003";
    if (write(sockfd, start_measurement, strlen(start_measurement)) < 0)
    {
        perror("mikes:tim571");
        mikes_log(ML_ERR, "writing start measurement packet to tim571");
    }
}

void skip_to_sentence_start()
{
	uint8_t readbuf;
    while (program_runs)
    {
        int nread = read(sockfd, &readbuf, 1);
        if (nread < 0)
        {
            perror("mikes:tim571");
            mikes_log(ML_ERR, "reading response from tim571");
            break;
        }
        if (readbuf == STX) return;
	}
}

static uint8_t sentence[MAX_SENTENCE_LENGTH];

void read_sentence()
{
	uint8_t *readbuf = sentence;
	skip_to_sentence_start();

    while (program_runs)
    {
        int nread = read(sockfd, readbuf, 1);
        if (nread < 0)
        {
            perror("mikes:tim571");
            mikes_log(ML_ERR, "reading response from tim571");
            break;
        }
        if (readbuf[0] == ETX) 
        {
			readbuf[0] = 0;
//			printf("TiM571: %s\n", sentence);
			return;
		}
		readbuf++;
	}	
}

static char *sp;
char *get_next_word()
{
	if (sp == 0) return "END";
	
	char *nxt = strchr(sp, ' ');
	char *wrd = sp;
	if (nxt != 0) 
	{
		*nxt = 0;
	    sp = nxt + 1;
	}
	else sp = 0;
	return wrd;
}

char *get_next_str()
{
	if (sp == 0) return "END";

	uint32_t str_len;
	sscanf(get_next_word(), "%x", &str_len);

	if (sp == 0) return "END";
	
	*(sp + str_len) = 0;
	char *wrd = sp;
	    
	sp += (str_len + 1);
	return wrd;
}

static uint16_t tim571_firmware;
static uint16_t tim571_sopas_device_id;
static uint32_t tim571_serial_num;
static uint32_t tim571_timestamp;
static uint8_t tim571_error;
static uint32_t tim571_telegram_counter;
static uint32_t tim571_scanning_frequency;
static float tim571_multiplier;
static int32_t tim571_starting_angle;
static uint16_t tim571_angular_step;
static uint16_t tim571_data_count;
static uint8_t tim571_rssi_channels;
static uint16_t *tim571_data;
static uint8_t *tim571_rssi_data;
static char tim571_name[18];

static uint8_t status_data_requested;
static uint8_t status_data_available;

void process_sentence()
{
	sp = (char *)sentence;
	char *token = get_next_word();
	
	if (strcmp(token, "sSN") != 0)
	{
		mikes_log_str(ML_ERR, "TiM571: unexpected start of TiM571 sentence: ", token); 
		return;
	}
	token = get_next_word();
	if (strcmp(token, "LMDscandata") != 0)
	{
		mikes_log_str(ML_ERR, "TiM571: unexpected data response string of TiM571 sentence: ", token); 
		return;
	}

	uint32_t tval32;

	token = get_next_word();
	sscanf(token, "%x", &tval32);

    if (status_data_requested)
        pthread_mutex_lock(&tim571_lock);

	tim571_firmware = (uint16_t)tval32;
	
	if (tim571_firmware != 1)
		mikes_log_str(ML_WARN, "TiM571: unexpected firmware version: ", token);
    
	token = get_next_word();
	sscanf(token, "%x", &tval32);
    tim571_sopas_device_id = tval32; 

	token = get_next_word();
	sscanf(token, "%x", &tim571_serial_num);

	token = get_next_word();
    if (strcmp(token, "0") != 0)
    {
		tim571_error = 1;
		mikes_log_str(ML_WARN, "TiM571: status1 is device error: ", token);
		if (status_data_requested) pthread_mutex_unlock(&tim571_lock);
		return;
	}

	token = get_next_word();
	if (strcmp(token, "0") != 0)
	{
		tim571_error = 1;
		mikes_log_str(ML_WARN, "TiM571: status2 is device error: ", token);
		if (status_data_requested) pthread_mutex_unlock(&tim571_lock);
	    return;
	}
	tim571_error = 0;
	
	uint32_t cnt;
	sscanf(get_next_word(), "%x", &cnt);
	if (tim571_telegram_counter && (cnt != tim571_telegram_counter + 1))
		mikes_log_double2(ML_WARN, "TiM571: out of order telegram counter: ", (double)cnt, (double)tim571_telegram_counter);
	tim571_telegram_counter = cnt;
	
	get_next_word(); // scan counter

	token = get_next_word();
	sscanf(token, "%x", &tim571_timestamp); // time since startup
	get_next_word(); // time of transmission
	get_next_word(); // input status
	get_next_word();
	get_next_word(); // output status
	get_next_word();
	get_next_word(); // reserved byte
	
	token = get_next_word();
	sscanf(token, "%x", &tim571_scanning_frequency);
	if (tim571_scanning_frequency != 1500) 
		mikes_log_str(ML_WARN, "TiM571: scanning frequency is not 1500: ", token);
	
	get_next_word(); // measurement frequency
	get_next_word(); // number of encoders
	get_next_word(); // number of channels
	
	token = get_next_word();
	if (strcmp(token, "DIST1") != 0)
	{
		mikes_log_str(ML_ERR, "TiM571: expected data header DIST1: ", token);
		if (status_data_requested) pthread_mutex_unlock(&tim571_lock);
		return;
	}
	
	token = get_next_word();
	uint32_t multiplier;
	sscanf(token, "%x", &multiplier);
	float *multiplier_as_float;
	multiplier_as_float = (float *)(&multiplier);
	tim571_multiplier = *multiplier_as_float;
	if (tim571_multiplier != 1.0)
		mikes_log_val(ML_WARN, "TiM571: unusual multiplier (1 expected): ", tim571_multiplier);
	
	get_next_word(); // scaling offset
	
	token = get_next_word();
	sscanf(token, "%x", &tval32);
	tim571_starting_angle = *((int32_t *)(&tval32));
	
	token = get_next_word();
	sscanf(token, "%x", &tval32);
	tim571_angular_step = (uint16_t)tval32;
	
	token = get_next_word();
	sscanf(token, "%x", &tval32);
	tim571_data_count = (uint16_t)tval32;
	
	for (int i = 0; i < tim571_data_count; i++)
	{
		token = get_next_word();
		sscanf(token, "%x", &tval32);
		tim571_data[i] = (uint16_t)tval32;
	}
	
	int will_send_RSSI;
	sscanf(get_next_word(), "%d", &will_send_RSSI);
	
	if (will_send_RSSI)
	{
		token = get_next_word();
		if (strcmp(token, "RSSI1") != 0)
		{
			mikes_log_str(ML_ERR, "TiM571: expected data header RSSI1: ", token);
			if (status_data_requested) pthread_mutex_unlock(&tim571_lock);
			return;
		}

		for (int i = 0; i < tim571_data_count; i++)
		{
			token = get_next_word();
			sscanf(token, "%x", &tval32);
			tim571_rssi_data[i] = (uint8_t)tval32;
		}
	}	
	tim571_rssi_channels = will_send_RSSI;
	
	get_next_word(); // no position data
	
	int will_send_name;
	sscanf(get_next_word(), "%d", &will_send_name);

	if (will_send_name)
	{
		token = get_next_str();
		strncpy(tim571_name, token, 17);
	}
	else tim571_name[0] = 0;

	get_next_word(); // no comment information	
	get_next_word(); // no time information
	get_next_word(); // no event information

    if (!status_data_requested)
        pthread_mutex_lock(&tim571_lock);

    memcpy(local_data, tim571_data, sizeof(uint16_t) * TIM571_DATA_COUNT);
    memcpy(local_rssi, tim571_rssi_data, sizeof(uint8_t) * TIM571_DATA_COUNT);

	if (status_data_requested) status_data_available = 1;
	pthread_mutex_unlock(&tim571_lock);
}

void *tim571_thread(void *args)
{
    send_start_measurement();
	read_sentence();

    while (program_runs)
    {
		read_sentence();
		process_sentence();
	}
    
    mikes_log(ML_INFO, "tim571 quits.");
    threads_running_add(-1);
    return 0;
}

void init_tim571()
{
    pthread_t t;
    tim571_data = (uint16_t *) malloc(sizeof(uint16_t) * TIM571_MAX_DATA_COUNT);
    tim571_rssi_data = (uint8_t *) malloc(sizeof(uint8_t) * TIM571_MAX_DATA_COUNT);
    local_data = (uint16_t *) malloc(sizeof(int) * TIM571_MAX_DATA_COUNT);
    local_rssi = (uint8_t *) malloc(sizeof(uint8_t) * TIM571_MAX_DATA_COUNT);
    tim571_error = 0;
	tim571_telegram_counter = 0;
	status_data_requested = 0;
	status_data_available = 0;
    if ((tim571_data == 0) || (local_data == 0) )
    {
      perror("mikes:tim571");
      mikes_log(ML_ERR, "insufficient memory");
      exit(1);
    }
    connect_tim571();
    pthread_mutex_init(&tim571_lock, 0);
    if (pthread_create(&t, 0, tim571_thread, 0) != 0)
    {
      perror("mikes:tim571");
      mikes_log(ML_ERR, "creating thread for tim571 sensor");
    }
    else threads_running_add(1);
}

void get_tim571_dist_data(uint16_t *buffer)
{
    pthread_mutex_lock(&tim571_lock);
    memcpy(buffer, local_data, sizeof(uint16_t) * TIM571_DATA_COUNT);
    pthread_mutex_unlock(&tim571_lock);
}

void get_tim571_rssi_data(uint8_t *buffer)
{
    pthread_mutex_lock(&tim571_lock);
    memcpy(buffer, local_rssi, sizeof(uint8_t) * TIM571_DATA_COUNT);
    pthread_mutex_unlock(&tim571_lock);
}

void get_tim571_status_data(tim571_status_data *status_data)
{
    pthread_mutex_lock(&tim571_lock);
	status_data_available = 0;
	status_data_requested = 1;
    pthread_mutex_unlock(&tim571_lock);	

	while (!status_data_available) usleep(10000);

    pthread_mutex_lock(&tim571_lock);
    
    status_data->firmware_version = tim571_firmware;
    status_data->sopas_device_id = tim571_sopas_device_id;
    status_data->serial_number = tim571_serial_num;
    status_data->error = tim571_error;
    status_data->scanning_frequency = tim571_scanning_frequency;
    status_data->multiplier = tim571_multiplier;
    status_data->starting_angle = tim571_starting_angle;
    status_data->angular_step = tim571_angular_step;
    status_data->data_count = tim571_data_count;
    status_data->rssi_available = tim571_rssi_channels;
    strncpy(status_data->name, tim571_name, 17);    

    pthread_mutex_unlock(&tim571_lock);	
}

void pretty_print_status_data(char *buffer, tim571_status_data *sd)
{
	sprintf(buffer, "TIM571 firmware version: %d\nSOPAS device id:         %d\nSerial number:           %d\nDevice error:            %d\nScanning frequency:      %u\nMultiplier:              %.3f\nStarting angle:          %d\nAngular step:            %u\nData count:              %d\nRSSI in data:            %d\n",
	                 sd->firmware_version, sd->sopas_device_id, sd->serial_number, sd->error, sd->scanning_frequency, sd->multiplier, sd->starting_angle, sd->angular_step, sd->data_count, sd->rssi_available);
}

int tim571_ray2azimuth(int ray)
{
  return (360 + (TIM571_TOTAL_ANGLE_DEG / 2 - ray / TIM571_SIZE_OF_ONE_DEG)) % 360;
}

int tim571_azimuth2ray(int alpha)
{
  if (360 - alpha <= TIM571_TOTAL_ANGLE_DEG / 2) alpha -= 360;
  if (alpha < -TIM571_TOTAL_ANGLE_DEG / 2) alpha = -TIM571_TOTAL_ANGLE_DEG / 2;
  else if (alpha > TIM571_TOTAL_ANGLE_DEG / 2) alpha = TIM571_TOTAL_ANGLE_DEG / 2;
  return TIM571_DATA_COUNT / 2 - alpha * TIM571_SIZE_OF_ONE_DEG;
}

void test_tim571()
{
   	uint16_t buffer[TIM571_DATA_COUNT];
    uint8_t buf2[TIM571_DATA_COUNT];
    char strbuf[3000];
    tim571_status_data status_data;
     
    get_tim571_dist_data(buffer); 
    get_tim571_rssi_data(buf2);  
    get_tim571_status_data(&status_data);
    pretty_print_status_data(strbuf, &status_data);    
    printf("%s\n", strbuf); 
    for (int i = 0; i < status_data.data_count; i++)
    {
	    if (i % 10 == 0) printf("\n%3d  ", i);
	    printf("%5u (%3u)  ", buffer[i], buf2[i]);
	}
	printf("\n---\n");
 }
 
