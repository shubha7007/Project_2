/*  Get_time

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include "protocol_examples_common.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "driver/hw_timer.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define BUF_SIZE (1024)

#define GPIO_OUTPUT_IO_0    CONFIG_AP_CON
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)
#define GPIO_OUTPUT_IO_1    CONFIG_RTC_NO
#define GPIO_OUTPUT_PIN_SEL_2  (1ULL<<GPIO_OUTPUT_IO_1)

#define NTP_TIMESTAMP_DELTA 2208988800ull

#define TEST_ONE_SHOT    false  
#define TEST_RELOAD      true

#define PORT CONFIG_TMP_PORT
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

typedef struct
{

	uint8_t li_vn_mode;
	uint8_t stratum;
	uint8_t poll;
	uint8_t precision;
	uint32_t rootDelay;
	uint32_t rootDispersion;
	uint32_t refId;
	uint32_t refTm_s;
	uint32_t refTm_f;
	uint32_t origTm_s;
	uint32_t origTm_f;       
	uint32_t rxTm_s;         
	uint32_t rxTm_f;         
	uint32_t txTm_s;        
	uint32_t txTm_f;         
} ntp_packet;              // Total: 384 bits or 48 bytes.

struct {
	char host_name[100];
	char zone[100];
	char format[5];
	int format_1;
	char offset_1[3];
	char offset_2[3];
	int offset_1i;
	int offset_2i;
	char sign;
} config_tmp;


char *pos;

#define TEST_ONE_SHOT    false  
#define TEST_RELOAD      true

static int state = 0;
static int state2 = 0;
static const char *TAG = "shubh";

static void udp_client_task(void *pvParameters)
{
	char addr_str[128];
	int addr_family;
	int ip_protocol;

	struct timeval read_timeout;
	read_timeout.tv_sec = 2;
	read_timeout.tv_usec = 10;


	struct tm * ptm;
	char buf[100]={0};
	ntp_packet packet = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	memset( &packet, 0, sizeof( ntp_packet ) );
	*( ( char * ) &packet + 0 ) = 0x1b; // Represents 27 in base 10 or 00011011 in base 2. 

	struct sockaddr_in destAddr;
	destAddr.sin_addr.s_addr = inet_addr(config_tmp.host_name);

	if(destAddr.sin_addr.s_addr == INADDR_NONE)
	{
		printf("\nEnter proper Server_ip\n");
		vTaskDelete(NULL);
	}

	destAddr.sin_family = AF_INET;
	destAddr.sin_port = htons(PORT);
	addr_family = AF_INET;
	ip_protocol = IPPROTO_IP;
	inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

	int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
	if (sock < 0) {
		printf("\nunable to create socket\n");
		vTaskDelete(NULL);
	}
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);	

	int err = sendto(sock, (char* ) &packet, sizeof(ntp_packet), 0, (struct sockaddr *) & destAddr, sizeof(destAddr));
	if (err < 0) {
		printf("\nunable to connect\n");
		vTaskDelete(NULL);
	}

	struct sockaddr_in sourceAddr; 
	socklen_t socklen = sizeof(sourceAddr);

	int len = recvfrom(sock, (char*) &packet, sizeof(ntp_packet), 0, (struct sockaddr *)&sourceAddr, &socklen);

	if (len < 1) {
		printf("\nunable to connect\n");
		vTaskDelete(NULL);
	}
	else {

		time_t txTm = ntohl(packet.txTm_s);
		txTm = (time_t)(txTm -NTP_TIMESTAMP_DELTA);

		if(config_tmp.sign == '+'){
			txTm = txTm + ((config_tmp.offset_1i * 60 + config_tmp.offset_2i)*60);
		}
		else if(config_tmp.sign == '-'){
			txTm = txTm - ((config_tmp.offset_1i * 60 + config_tmp.offset_2i)*60);
		}

		ptm = localtime(&txTm);

		switch(config_tmp.format_1)
		{
			case 1:
				strftime(buf, 256, "%G-%m-%d", ptm);
				printf("\n%s",buf);
				printf("  %2d:%02d:%02d\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
				break;
			case 2:
				strftime(buf, 256, "%y-%m-%d", ptm);
				printf("\n%s",buf);
				printf ("  %2d:%02d:%02d\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
				break;
			case 3:
				strftime(buf, 256, "%G-%h-%d <%p>", ptm);
				printf("\n%s",buf);
				printf ("  %2d:%02d:%02d\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
				break;
			case 4:
				printf ("\n%2d:%02d:%02d", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
				strftime(buf, 256, "  %d %h %G\n", ptm);
				printf("%s",buf);
				break;

			default:
				printf("wrong format");
		}


	}

	vTaskDelay(2000 / portTICK_PERIOD_MS);

	if (sock != -1) {
		ESP_LOGE(TAG, "Shutting down socket and restarting...");
		shutdown(sock, 0);
		close(sock);
	}
	vTaskDelete(NULL);
}



static void event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {

			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			gpio_set_level(GPIO_OUTPUT_IO_0,0);

		}
		ESP_LOGI(TAG,"connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI(TAG, "got ip:%s",
				ip4addr_ntoa(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void wifi_init_sta(void)
{
	s_wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();

	ESP_ERROR_CHECK(esp_event_loop_create_default());

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = EXAMPLE_ESP_WIFI_SSID,
			.password = EXAMPLE_ESP_WIFI_PASS
		},
	};

	if (strlen((char *)wifi_config.sta.password)) {
		wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
	}

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
	ESP_ERROR_CHECK(esp_wifi_start() );

	ESP_LOGI(TAG, "wifi_init_sta finished.");

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
	 * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
			WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
			pdFALSE,
			pdFALSE,
			portMAX_DELAY);

	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
				EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
				EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
		gpio_set_level(GPIO_OUTPUT_IO_0,0);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
		gpio_set_level(GPIO_OUTPUT_IO_0,0);
	}
}

static void echo_task()
{

	char cmd[81];
	int i=0,n=0,s=0,j=0;
	char target[10][25];
	int error=0;

	// Configure parameters of an UART driver,
	// communication pins and install the driver
	uart_config_t uart_config = {
		.baud_rate = 74880,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(UART_NUM_0, &uart_config);
	uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

	uint8_t *ch = (uint8_t *) malloc(BUF_SIZE);


	strcpy(config_tmp.host_name,CONFIG_TMP_IP);

	strcpy(config_tmp.zone,CONFIG_TMP_ZONE);

	config_tmp.format_1 = CONFIG_TMP_FORMAT;

	while (1) {
		// Read data from the UART
		int len = uart_read_bytes(UART_NUM_0, ch, BUF_SIZE, 20 / portTICK_RATE_MS);

		if(len!=0){
			if(*ch == '\n' || *ch == '\r'){
				memset(target,0,sizeof(target));				
				cmd[i] = 0;
				i = 0;
				n=0;
				for(s=0; 1; s++)
				{

					if(cmd[s] != ' ' && cmd[s]!= '\0' && cmd[s] != 0 ){
						//  storing each letter
						target[n][j++] = cmd[s];
					}
					else{
						//  storing each word
						target[n][j++]='\0';
						n++;
						j=0;
					}

					if(cmd[s]== '\0' || cmd[s] == 0)
					{

						strcpy(config_tmp.host_name,CONFIG_TMP_IP);
						strcpy(config_tmp.zone,CONFIG_TMP_ZONE);

						if(CONFIG_TMP_FORMAT>0 && CONFIG_TMP_FORMAT <5){
							config_tmp.format_1 = CONFIG_TMP_FORMAT;
						}
						else{
							printf("\nEnter correct Format\n");
							break;
						}

						pos = strstr(config_tmp.zone,"UTC");
						if(pos==NULL){
							printf("\nEnter correct timezone-2\n");
							error=1;
							break;
						}
						pos = pos+3;
						if(*pos=='+'){
							config_tmp.sign='+';
						}
						else if(*pos=='-'){
							config_tmp.sign='-';
						}
						else{
							printf("\nEnter correct timezone\n");
							error=1;
							break;
						}

						if((*(pos+1) >= 48 && *(pos+1) <= 50)){
							config_tmp.offset_1[0]=*(pos+1);
						}
						else{
							printf("\nEnter correct timezone\n");
							break;
						}
						if(*(pos+2) >= 48 && *(pos+2) <= 57){
							config_tmp.offset_1[1]=*(pos+2);
						}else
						{
							printf("\nEnter correct timezone\n");
							break;	
						}
						config_tmp.offset_1[2]='\0';

						pos = strstr(config_tmp.zone,":");
						if(*(pos+1) >= 48 && *(pos+1) <= 54){
							config_tmp.offset_2[0]=*(pos+1);
						}
						else{
							printf("\nEnter correct timezone\n");
							break;
						}
						if(*(pos+2) >= 48 && *(pos+2) <= 57){
							config_tmp.offset_2[1]=*(pos+2);
						}
						else{
							printf("\nenter correct timezone\n");
							break;
						}
						config_tmp.offset_2[2]='\0';

						config_tmp.offset_1i = atoi(config_tmp.offset_1);
						config_tmp.offset_2i = atoi(config_tmp.offset_2);
						if(strcmp(cmd,"get_time")==0){
							xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
							break;
						}

						else if(strcmp(target[0],"get_time")==0){
							for(s=0; s<n ;s++)
							{
								if(strcmp(target[s],"-s")==0)
								{
									strcpy(config_tmp.host_name,target[s+1]);

									if(INADDR_NONE==inet_addr(config_tmp.host_name))
									{
										printf("\nEnter correct Server_ip\n");
										error=1;
										break;
									}
								}
								else if(strcmp(target[s],"-z")==0)
								{
									strcpy(config_tmp.zone,target[s+1]);


									if((target[s+1][0]!= 'U') && target[s+1][1]!='T' && target[s+1][2]!= 'C'){
										printf("\n Enter correct timezone\n");
										error=1;
										break;
									}


									pos = strstr(config_tmp.zone,"UTC");
									if(pos==NULL){
										printf("\nEnter correct timezone\n");
										error=1;
										break;
									}
									pos = pos+3;

									if(*pos=='+'){
										config_tmp.sign='+';
									}
									else if(*pos=='-'){
										config_tmp.sign='-';
									}
									else{
										printf("\nEnter correct timezone\n");
										error=1;
										break;
									}
									if((*(pos+1) >= 48 && *(pos+1) <= 50)){
										config_tmp.offset_1[0]=*(pos+1);
									}
									else{
										printf("\nEnter correct timezone\n");
										error=1;
										break;
									}
									if(*(pos+2) >= 48 && *(pos+2) <= 57){
										config_tmp.offset_1[1]=*(pos+2);
									}else
									{
										printf("\nEnter correct timezone\n");
										error=1;
										break;
									}
									config_tmp.offset_1[2]='\0';

									if(*(pos+3)!=':'){
										printf("\n Enter correct timezone\n");
										error=1;
										break;
									}

									pos = strstr(config_tmp.zone,":");
									if(*(pos+1) >= 48 && *(pos+1) <= 54){
										config_tmp.offset_2[0]=*(pos+1);
									}
									else{
										printf("\nEnter correct timezone\n");
										error=1;
										break;
									}
									if(*(pos+2) >= 48 && *(pos+2) <= 57){
										config_tmp.offset_2[1]=*(pos+2);
									}
									else{
										printf("\nEnter correct timezone\n");
										error=1;
										break;
									}
									config_tmp.offset_2[2]='\0';
									if(*(pos+3)!='\0'){
										printf("\n Enter correct timezone\n");
										error=1;
										break;
									}


									config_tmp.offset_1i = atoi(config_tmp.offset_1);
									config_tmp.offset_2i = atoi(config_tmp.offset_2);

								}
								else if(strcmp(target[s],"-f")==0)
								{
									strcpy(config_tmp.format,target[s+1]);

									config_tmp.format_1=atoi(config_tmp.format);
									if(config_tmp.format_1 <1 || config_tmp.format_1 > 4){
										printf("\nEnter correct Format\n");
										error=1;
										break;
									}
								}
							}
							if(error == 1)
							{
								error=0;
								break;
							}
							xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

							n=0;
							break;
						}
						else{
							n=0;
							printf("\nPlease enter cmd properly as given below\n");
							printf("get_time -s server_ip -z timezone -f format\n");
							printf("EXAMPLE: get_time -s 66.151.147.38 -z UTC+05:30 -f 1\n");
							break;
						}
					}
				}

			}
			else{
				if(i < sizeof(cmd)){
					cmd[i++] = *ch;
					uart_write_bytes(UART_NUM_0, (const char *)ch, 1);
				}
			}

		}

	}
}

void hw_timer_callback1(void *arg)
{

	state++;
	if(state==60)
	{
		state =0;

		/****   get time and updating RTC code will come here ****/
		/*** error replace 0 with 1 in below command    ****/
		if(0){
			state2++;
		}
		else{ 
			state2=0;
		}
	}
	else if(state2==5)
	{
		state2 = 0;
		gpio_set_level(GPIO_OUTPUT_IO_1,0);
	}
}


static void timer_reset_task()
{
	ESP_LOGI(TAG, "Initialize hw_timer for callback1");
	hw_timer_init(hw_timer_callback1, NULL);
	ESP_LOGI(TAG, "Set hw_timer timing time 100us with reload");
	hw_timer_alarm_us(1000*1000,TEST_RELOAD);
	vTaskDelay(1000*1000 / portTICK_RATE_MS);
}

static void gpio_init()
{
	gpio_config_t io_conf;

	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;

	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	io_conf.pin_bit_mask |= GPIO_OUTPUT_PIN_SEL_2;

	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_set_level(GPIO_OUTPUT_IO_0,1);
	gpio_set_level(GPIO_OUTPUT_IO_1,1);
}

void app_main()
{

	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	gpio_init();
	wifi_init_sta();

	xTaskCreate(echo_task, "uart_echo_task", 4096, NULL, 10, NULL);

	xTaskCreate(timer_reset_task, "timer_reset", 1024, NULL, 10, NULL);
}
