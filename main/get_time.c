/* Hello World Example

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

///////////////
#include "driver/uart.h"
#define BUF_SIZE (1024)
///////////////
#include "driver/gpio.h"
#define GPIO_OUTPUT_IO_0    2
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)
///////////////
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

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define NTP_TIMESTAMP_DELTA 2208988800ull

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
} config_tmp;

char *pos;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <sys/param.h>
#include "protocol_examples_common.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

//#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR "66.151.147.38"
//#else
//#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
//#endif

#define CONFIG_EXAMPLE_PORT 123
#define PORT CONFIG_EXAMPLE_PORT

static const char *payload = "Message from ESP32 ";

#define EXAMPLE_ESP_WIFI_SSID      "Redmi_k20"
#define EXAMPLE_ESP_WIFI_PASS      "12345777"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "driver/hw_timer.h"
#define TEST_ONE_SHOT    false  
#define TEST_RELOAD      true
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    static int state = 0;
    static int state2 = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void udp_client_task(void *pvParameters)
{
	char addr_str[128];
	int addr_family;
	int ip_protocol;

	struct tm * ptm;
	struct tm ptm1;
	char buf[100]={0};
	ntp_packet packet = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	memset( &packet, 0, sizeof( ntp_packet ) );
	*( ( char * ) &packet + 0 ) = 0x1b; // Represents 27 in base 10 or 00011011 in base 2. 

	//#ifdef CONFIG_EXAMPLE_IPV4
	struct sockaddr_in destAddr;
	//destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
	destAddr.sin_addr.s_addr = inet_addr(config_tmp.host_name);
	destAddr.sin_family = AF_INET;
	destAddr.sin_port = htons(PORT);
	addr_family = AF_INET;
	ip_protocol = IPPROTO_IP;
	inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
	//#else  IPV6
	/*
	   struct sockaddr_in6 destAddr;
	   inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
	   destAddr.sin6_family = AF_INET6;
	   destAddr.sin6_port = htons(PORT);
	   destAddr.sin6_scope_id = tcpip_adapter_get_netif_index(TCPIP_ADAPTER_IF_STA);
	   addr_family = AF_INET6;
	   ip_protocol = IPPROTO_IPV6;
	   inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
	 */
	//#endif

	int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
	if (sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		//  break;
	}

	int err = sendto(sock, (char* ) &packet, sizeof(ntp_packet), 0, (struct sockaddr *) & destAddr, sizeof(destAddr));
	if (err < 0) {
		ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
		//               break;
	}
	ESP_LOGI(TAG, "Message sent");

	struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
	socklen_t socklen = sizeof(sourceAddr);
	int len = recvfrom(sock, (char*) &packet, sizeof(ntp_packet), 0, (struct sockaddr *)&sourceAddr, &socklen);

	// Error occured during receiving
	if (len < 0) {
		ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
		//break;
	}
	// Data received
	else {

		time_t txTm = ntohl(packet.txTm_s);
		txTm = (time_t)(txTm -NTP_TIMESTAMP_DELTA);
		//txTm = txTm + ((5 * 60 + 30)*60);


		uart_write_bytes(UART_NUM_0, (const char *)config_tmp.offset_1, strlen(config_tmp.offset_1));
		//uart_write_bytes(UART_NUM_0, (const char *)config_tmp.offset_2, strlen(config_tmp.offset_2));

		txTm = txTm + ((config_tmp.offset_1i * 60 + config_tmp.offset_2i)*60);
		ptm = localtime(&txTm);

		switch(config_tmp.format_1)
		{
			case 1:
				strftime(buf, 256, "%G-%m-%d", ptm);
				printf("%s",buf);
				printf("  %2d:%02d:%02d\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
				break;
			case 2:
				strftime(buf, 256, "%y-%m-%d", ptm);
				printf("%s",buf);
				printf ("  %2d:%02d:%02d\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
				break;
			case 3:
				strftime(buf, 256, "%G-%h-%d <%p>", ptm);
				printf("%s",buf);
				printf ("  %2d:%02d:%02d\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
				break;
			case 4:
				printf ("%2d:%02d:%02d", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
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
    		gpio_set_level(2,0);

            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
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
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

//    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
//    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
//    vEventGroupDelete(s_wifi_event_group);
}

static void echo_task()
{

	char cmd[81];
	int i=0;

	////////////////////////////////////////////////////////////
	int n=0,s=0,j=0;
	char target[10][25];
	////////////////////////////////////////////////////////////


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

	// Configure a temporary buffer for the incoming data
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	uint8_t *ch = (uint8_t *) malloc(BUF_SIZE);


//	config_tmp.host_name = CONFIG_TMP_IP;
	strcpy(config_tmp.host_name,CONFIG_TMP_IP);
	uart_write_bytes(UART_NUM_0, (const char *)config_tmp.host_name, strlen(config_tmp.host_name));

//	config_tmp.zone = CONFIG_TMP_ZONE;
	strcpy(config_tmp.zone,CONFIG_TMP_ZONE);
	uart_write_bytes(UART_NUM_0, (const char *)config_tmp.zone, strlen(config_tmp.zone));

	config_tmp.format_1 = CONFIG_TMP_FORMAT;

	while (1) {
		// Read data from the UART
		int len = uart_read_bytes(UART_NUM_0, ch, BUF_SIZE, 20 / portTICK_RATE_MS);

		if(len!=0){
			if(*ch == '\n' || *ch == '\r'){
				//cmd[i++] = '\0';
				cmd[i] = 0;
				i = 0;

				//uart_write_bytes(UART_NUM_0, (const char *)cmd, i);

				if(strcmp(cmd,"./get_time")==0){
				//	xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
				}

				for(s=0; 1; s++)
				{
					//if(cmd[s] != ' ' || cmd[s] != '\0' || cmd[s] != 0 ){
					if(cmd[s] != ' ' && cmd[s]!= '\0' && cmd[s] != 0 ){
						target[n][j++] = cmd[s];
						//uart_write_bytes(UART_NUM_0, (const char *)target[n], j);
					}
					else{
						target[n][j++]='\0';
						//printf("ya %s:",target[n]);
						//uart_write_bytes(UART_NUM_0, (const char *)target[n], j);
						n++;
						j=0;
						if(cmd[s]== '\0' || cmd[s] == 0)
						{
					
					for(s=0; s<n ;s++)
					{
                                                if(strcmp(cmd,"./get_time")==0){
                                                pos = strstr(config_tmp.zone,"UTC");
                                                pos = pos+3;
                                                config_tmp.offset_1[0]=*(pos+1);
                                                config_tmp.offset_1[1]=*(pos+2);
                                                config_tmp.offset_1[2]='\0';

                                                pos = strstr(config_tmp.zone,":");
                                                config_tmp.offset_2[0]=*(pos+1);
                                                config_tmp.offset_2[1]=*(pos+2);
                                                config_tmp.offset_2[2]='\0';
						xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
						}

						if(strstr(target[s],"-s"))
						{
						//uart_write_bytes(UART_NUM_0, (const char *)target[s+1], strlen(target[s+1]));
						strcpy(config_tmp.host_name,target[s+1]);
						uart_write_bytes(UART_NUM_0, (const char *)config_tmp.host_name, strlen(config_tmp.host_name));
						}
						else if(strstr(target[s],"-z"))
						{
						//uart_write_bytes(UART_NUM_0, (const char *)target[s+1], strlen(target[s+1]));
						strcpy(config_tmp.zone,target[s+1]);
						uart_write_bytes(UART_NUM_0, (const char *)config_tmp.zone, strlen(config_tmp.zone));

///////////////////////////////////////////////////////////////////////////////////////////////////

						pos = strstr(config_tmp.zone,"UTC");
						pos = pos+3;
						config_tmp.offset_1[0]=*(pos+1);
						config_tmp.offset_1[1]=*(pos+2);
						config_tmp.offset_1[2]='\0';

						pos = strstr(config_tmp.zone,":");
						config_tmp.offset_2[0]=*(pos+1);
						config_tmp.offset_2[1]=*(pos+2);
						config_tmp.offset_2[2]='\0';;

					//	uart_write_bytes(UART_NUM_0, (const char *)config_tmp.offset_1, strlen(config_tmp.offset_1));
					//	uart_write_bytes(UART_NUM_0, (const char *)config_tmp.offset_2, strlen(config_tmp.offset_2));
						
						config_tmp.offset_1i = atoi(config_tmp.offset_1);
						config_tmp.offset_2i = atoi(config_tmp.offset_2);

///////////////////////////////////////////////////////////////////////////////////////////////////


						}
						else if(strstr(target[s],"-f"))
						{
						//uart_write_bytes(UART_NUM_0, (const char *)target[s+1], strlen(target[s+1]));
						strcpy(config_tmp.format,target[s+1]);
						config_tmp.format_1=atoi(config_tmp.format);
						uart_write_bytes(UART_NUM_0, (const char *)config_tmp.format, strlen(config_tmp.format));
						}
					}



							n=0;
							//printf("ya %s:",target[n]);
							//uart_write_bytes(UART_NUM_0, (const char *)target[n], j);
							break;
						}
						//break;
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

// if want here will come rtc code
    if(1){
    state2++;
    }
    else{ 
    state2=0;
    }
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    //gpio_set_level(GPIO_OUTPUT_IO_0,1);
}
else if(state2==5)
{
    state2 = 0;
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    ///gpio_set_level(GPIO_OUTPUT_IO_0,0);
}
}


static void timer_reset_task()
{
	
    gpio_config_t io_conf;
    //disable interrupt 
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(2,1);

    ESP_LOGI(TAG, "Initialize hw_timer for callback1");
    hw_timer_init(hw_timer_callback1, NULL);
    ESP_LOGI(TAG, "Set hw_timer timing time 100us with reload");
    hw_timer_alarm_us(1000*1000,TEST_RELOAD);
    vTaskDelay(1000*1000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Deinitialize hw_timer for callback1");
}

void app_main()
{

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    //xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
    xTaskCreate(echo_task, "uart_echo_task", 4096, NULL, 10, NULL);

//    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    xTaskCreate(timer_reset_task, "timer_reset", 1024, NULL, 10, NULL);
}










