// --------------------------------------------------------------------------
// debug support
// --------------------------------------------------------------------------

static const char *TAG = "app_BB3";
#include "esp_log.h"

// #define DBG_MEMLEAK
// #define DBG_HEAP_INFO

// #define NO_DBG_MEMLEAK
// #define NO_DBG_HEAP_INFO

#include "dbg_mem.h"
#include "dbg_helper.h"          // HEAP_INFO()

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
// #include <unistd.h>
#include <sys/socket.h>          // shutdown(),  close()
#include <errno.h>
// #include <netdb.h>            // struct addrinfo
// #include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR "192.168.100.80"
#define PORT 34089

static const char *CH1_STATUS = "OUTP? CH1\n";
static const char *CH2_STATUS = "OUTP? CH2\n";

static const char *CH1_ON = "OUTP ON, CH1\n";
static const char *CH1_OFF = "OUTP OFF, CH1\n";
static const char *CH2_ON = "OUTP ON, CH2\n";
static const char *CH2_OFF = "OUTP OFF, CH2\n";

static const char *CH_ALL_STAT = "MEAS:VOLT? CH1;:MEAS:CURR? CH1;:MEAS:VOLT? CH2;:MEAS:CURR? CH2\n";
char ALL_CH_STAT[50] = "";

extern char CH1_V[];
extern char CH1_A[];
extern char CH2_V[];
extern char CH2_A[];
extern char WIFI_SSID[];
extern char WIFI_PASS[];
extern char WIFI_IP[];
extern char BB3_IP[];
extern bool BB3_CONNECTED;
extern bool WIFI_CONNECTED;
extern bool SCPI_UPDATED;
extern bool CH1_UPDATE;
extern bool CH1_POWER_ON;
extern bool CH2_UPDATE;
extern bool CH2_POWER_ON;
extern char BB3_RESPONSE[];

// --------------------------------------------------------------------------
// local variables
// --------------------------------------------------------------------------

// esp netif object representing the WIFI station
esp_netif_t *sta_netif = NULL;

SemaphoreHandle_t connectionSemaphore;

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

static void ICACHE_FLASH_ATTR event_handler( void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data )
{
   if (event_base == WIFI_EVENT )
   {
      switch (event_id) {
        case WIFI_EVENT_STA_START:
          esp_wifi_connect();
          ESP_LOGI(TAG,"connecting...\n");
          break;

        case WIFI_EVENT_STA_CONNECTED:
          ESP_LOGI(TAG,"connected\n");
          break;

        case WIFI_EVENT_STA_DISCONNECTED:
          ESP_LOGI(TAG,"disconnected\n");
          break;

        default:
          ESP_LOGI(TAG,"Wifi event %d", event_id );
          break;
      }
   }
   else if( event_base == IP_EVENT )
   {
     switch (event_id) {
       case IP_EVENT_STA_GOT_IP:
         ESP_LOGI(TAG,"got ip\n");
         xSemaphoreGive(connectionSemaphore);
         break;

       default:
         ESP_LOGI(TAG,"IP event %d", event_id );
         break;
     }
   }
}

void wifiInit()
{
   ESP_LOGI( TAG, "Wifi initialize ..." );

   ESP_ERROR_CHECK(nvs_flash_init()); //inicialización de memoria flash
   // initialize the tcp stack
   ESP_ERROR_CHECK(esp_netif_init());
   // initialize the wifi event handler
   // use one handler for all events for the given type
   ESP_ERROR_CHECK( esp_event_loop_create_default() );

  // Initialize default station as network interface instance (esp-netif)
  sta_netif = esp_netif_create_default_wifi_sta();

   // initialize wifi stack
  wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
   wifi_config_t wifi_config = {
     .sta = {
         // .ssid = WIFI_SSID,
         // .password = WIFI_PASS,
         /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
          * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
          * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
          * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
          */
         .threshold = {
            .authmode = WIFI_AUTH_OPEN,
         },
         .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
         .sae_h2e_identifier = "",
     },
  };
  memcpy(wifi_config.sta.ssid,WIFI_SSID,strlen(WIFI_SSID));
  memcpy(wifi_config.sta.password,WIFI_PASS,strlen(WIFI_PASS));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI( TAG, "Wifi initialize done" );
}

static void parseCH_ALL_STAT(){
  ESP_LOGI( TAG, "RECVEIVED ALL CH STATUS: %s", BB3_RESPONSE);

  //char    str[]= "ls -l";
  char ** res  = NULL;
  char *  p    = strtok (BB3_RESPONSE, ";");
  int n_spaces = 0;


  /* split string and append tokens to 'res' */

  while (p) {
    res = ( char **)realloc (res, sizeof (char*) * ++n_spaces);

    if (res == NULL)
      exit (-1); /* memory allocation failed */

    res[n_spaces-1] = p;

    p = strtok (NULL, ";");
  }

  /* realloc one extra element for the last NULL */

  res = ( char **)realloc (res, sizeof (char*) * (n_spaces+1));
  res[n_spaces] = 0;
  ESP_LOGI( TAG, "Got %i parameter/s %d", n_spaces, sizeof(res[0] ));
  if (n_spaces == 4) {
    /* print the result */
    if (sizeof(res[0]) == 1){sprintf(CH1_V, "0.0000");}else{sprintf(CH1_V, res[0]);}
    if (sizeof(res[1]) == 1){sprintf(CH1_A, "0.0000");}else{sprintf(CH1_A, res[1]);}
    if (sizeof(res[2]) == 1){sprintf(CH2_V, "0.0000");}else{sprintf(CH2_V, res[2]);}
    if (sizeof(res[3]) == 1){sprintf(CH2_A, "0.0000");}else{sprintf(CH2_A, res[3]);}
  }
  /* free the memory allocated */

  free (res);
}

static void tcp_client_task(void *pvParameters)
{
  ESP_LOGI( TAG, "Run tcp client task" );

  char rx_buffer[128];
  char host_ip[] = HOST_IP_ADDR;
  int addr_family = 0;
  int ip_protocol = 0;
  while (1) {
    if(SCPI_UPDATED) {
      SCPI_UPDATED= false;
      BB3_CONNECTED = false;
      ESP_LOGI( TAG, "SCPI update: ");
      if (WIFI_CONNECTED == true) {
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
          ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
          SCPI_UPDATED= false;
          BB3_CONNECTED = false;
          CH1_UPDATE = false;
          CH2_UPDATE = false;
          //break;
        }
        else {
          ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);
          BB3_CONNECTED = true;
        }
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
          ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
          SCPI_UPDATED= false;
          BB3_CONNECTED = false;
          CH1_UPDATE = false;
          CH2_UPDATE = false;
          //break;
        }
        else {
          ESP_LOGI(TAG, "Successfully connected");
          BB3_CONNECTED = true;
        }
        if (BB3_CONNECTED) {
          while (1) {
            int err = 0;
            ESP_LOGI( TAG, "Connected to BB3 and waiting for SCPI commands");
            SCPI_UPDATED = false;
            BB3_CONNECTED = false;
            if(CH1_UPDATE) {
              ESP_LOGI( TAG, "CH1 update");
              SCPI_UPDATED= false;
              BB3_CONNECTED = false;
              CH1_UPDATE = false;
              CH2_UPDATE = false;
              if(CH1_POWER_ON) {err = send(sock, CH1_ON, strlen(CH1_ON), 0);}
              else {err = send(sock, CH1_OFF, strlen(CH1_OFF), 0);}
              vTaskDelay(1 / portTICK_PERIOD_MS);
              err = send(sock, CH_ALL_STAT, strlen(CH_ALL_STAT), 0);
            }
            else if(CH2_UPDATE) {
              ESP_LOGI( TAG, "CH2 update");
              SCPI_UPDATED= false;
              BB3_CONNECTED = false;
              CH1_UPDATE = false;
              CH2_UPDATE = false;
              if(CH2_POWER_ON) {err = send(sock, CH2_ON, strlen(CH2_ON), 0);}
              else {err = send(sock, CH2_OFF, strlen(CH2_OFF), 0);}
              vTaskDelay(1 / portTICK_PERIOD_MS);
              err = send(sock, CH_ALL_STAT, strlen(CH_ALL_STAT), 0);
            }
            // Get status of BB3
            else {
              err = send(sock, CH_ALL_STAT, strlen(CH_ALL_STAT), 0);
              ESP_LOGI( TAG, "GET ALL CH STATUS: %s", CH_ALL_STAT);
              SCPI_UPDATED= false;
              BB3_CONNECTED = false;
              CH1_UPDATE = false;
              CH2_UPDATE = false;
            }

            if (err < 0) {
              ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
              SCPI_UPDATED= false;
              BB3_CONNECTED = false;
              CH1_UPDATE = false;
              CH2_UPDATE = false;
              break;
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
              ESP_LOGE(TAG, "recv failed: errno %d", errno);
              SCPI_UPDATED = false;
              BB3_CONNECTED = false;
              CH1_UPDATE = false;
              CH2_UPDATE = false;
              // break;
            }
            // Data received
            else {
              rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
              sprintf(BB3_RESPONSE, rx_buffer);
              ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
              ESP_LOGI(TAG, "%s", rx_buffer);
              parseCH_ALL_STAT();
              SCPI_UPDATED= false;
              BB3_CONNECTED = false;
              CH1_UPDATE = false;
              CH2_UPDATE = false;
              shutdown(sock, 0);
              close(sock);
              break;
            }
            //}
            vTaskDelay(100 / portTICK_PERIOD_MS);
          }
        }
        if (sock != -1) {
          ESP_LOGE(TAG, "Shutting down socket and restarting...");
          CH1_UPDATE = false;
          CH2_UPDATE = false;
          BB3_CONNECTED = false;
          SCPI_UPDATED= false;
          shutdown(sock, 0);
          close(sock);
        }
        SCPI_UPDATED= false;
        CH1_UPDATE = false;
        CH2_UPDATE = false;
        BB3_CONNECTED = false;
        shutdown(sock, 0);
        close(sock);
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void OnConnected(void *para)
{
  ESP_LOGI( TAG, "Run OnConnected task" );

  while (true) {
    if (xSemaphoreTake(connectionSemaphore, 10000 / portTICK_PERIOD_MS)) {
      ESP_LOGI(TAG, "connected");
      WIFI_CONNECTED = true;
      // do something
      esp_netif_ip_info_t ip_info;
      ESP_ERROR_CHECK(esp_netif_get_ip_info(sta_netif,&ip_info));
      ESP_LOGI( TAG, "IP Addresss: %s", ip4addr_ntoa((const ip4_addr_t*)&ip_info.ip.addr));
      ESP_LOGI( TAG, "Subnet Mask: %s", ip4addr_ntoa((const ip4_addr_t*)&ip_info.netmask.addr));
      ESP_LOGI( TAG, "Gateway: %s", ip4addr_ntoa((const ip4_addr_t*)&ip_info.ip.addr));
      sprintf(WIFI_IP,ip4addr_ntoa((const ip4_addr_t*)&ip_info.ip.addr));
      ESP_LOGI( TAG, "IP: %s", WIFI_IP);
      xSemaphoreTake(connectionSemaphore, portMAX_DELAY);
    }
    else {
      WIFI_CONNECTED = false;
      ESP_LOGE(TAG, "Failed to connect. Retry in");
      for (int i = 0; i < 10000; i++) {
        ESP_LOGE(TAG, "...%d", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
      esp_restart(); // si no hay conexion reiniciar el chip.
    }
  }
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

extern "C"
void appBB3( void )
{
   esp_log_level_set(TAG, ESP_LOG_DEBUG);
   connectionSemaphore = xSemaphoreCreateBinary();
   wifiInit();
   xTaskCreate(&OnConnected, "handel comms", 1024 * 4, NULL, 5, NULL);
   xTaskCreate(&tcp_client_task, "tcp_client_task", 4096 * 3,NULL,2,NULL);
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

