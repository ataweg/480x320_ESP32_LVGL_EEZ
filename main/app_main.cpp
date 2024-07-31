// --------------------------------------------------------------------------
//
// Project       Wifi-WeatherStation
//
// File          app_main.c
//
// Author        Axel Werner
//
// --------------------------------------------------------------------------
// Changelog
//
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// debug support
// --------------------------------------------------------------------------

static const char *TAG = "480x320_ESP32_LVGL_EEZ";
#include "esp_log.h"

// #define DBG_MEMLEAK
#define DBG_HEAP_INFO

// #define NO_DBG_MEMLEAK
// #define NO_DBG_HEAP_INFO

#include "dbg_mem.h"
#include "dbg_helper.h"          // HEAP_INFO()

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#include <string.h>
#include <stdio.h>
#include <sys/time.h>               // struct timeval
#include <esp32/rom/rtc.h>          // rtc_get_wakeup_cause()

#include "freertos/FreeRTOS.h"      // xPortGetFreeHeapSize()
#include "freertos/task.h"          // vTaskList()
#include "freertos/event_groups.h"

#include "esp_chip_info.h"          // esp_chip_info_t
#include "esp_mac.h"                // esp_read_mac()
#include "esp_netif.h"              // IPSTR, IP2STR()
#include "esp_heap_caps.h"          // heap_caps_get_largest_free_block(), heap_caps_get_minimum_free_size()
#include "esp_private/esp_clk.h"    // esp_clk_cpu_freq()
#include "nvs_flash.h"              // nvs_flash_init()
#include "esp_flash.h"              // esp_flash_get_size()

#ifdef CONFIG_IDF_CMAKE
   #include "build_info.h"          // __BUILD_NUMBER, __BUILD_UTIME
#endif

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

extern "C" void appGui( void );
extern "C" void appBB3( void );

// --------------------------------------------------------------------------
// heap tracing
// --------------------------------------------------------------------------
/*
 * esp_err_t heap_trace_init_standalone( heap_trace_record_t *record_buffer, size_t num_records );
 * esp_err_t heap_trace_start( heap_trace_mode_t mode );
 * esp_err_t heap_trace_stop( void );
 - esp_err_t heap_trace_resume( void );
 - size_t heap_trace_get_count( void );
 - esp_err_t heap_trace_get( size_t index, heap_trace_record_t *record );
 * void heap_trace_dump( void );
*/

#ifdef CONFIG_HEAP_TRACING
   #define TRACE_ENABLE

   #ifdef TRACE_ENABLE
      #include "esp_heap_trace.h"

      #define NUM_RECORDS 64
      static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM
   #endif
#endif

// --------------------------------------------------------------------------
// build time variables
// --------------------------------------------------------------------------

// Note that linker symbols are not variables, they have no memory allocated for
// maintaining a value, rather their address is their value.

#ifdef CONFIG_IDF_CMAKE
   extern const time_t   build_utime  = ( time_t ) __BUILD_UTIME;
   extern const uint32_t build_number = __BUILD_NUMBER;
#else
   extern char __BUILD_UTIME[];    // __BUILD_UTIME=$$(date +'%s')
   extern char __BUILD_NUMBER[];   // __BUILD_NUMBER=$$(cat $(BUILD_NUMBER_FILE))

   extern const time_t   build_utime  = ( time_t ) &__BUILD_UTIME;
   extern const uint32_t build_number = ( uint32_t ) &__BUILD_NUMBER;
#endif

// --------------------------------------------------------------------------
// Hardware resources
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Prototypes
// --------------------------------------------------------------------------

extern "C" void printSystemInfo( void );
extern "C" void app_main( void );
extern "C" void printHeapInfo( void );
extern "C" const char* rtc_reset_reason2Str( int reason );
extern "C" const char* reset_reason2Str( int reason );
extern "C" void printTaskList( void );

static void PinsInit( void );

// --------------------------------------------------------------------------
// global variables
// --------------------------------------------------------------------------

esp_reset_reason_t reason_esp_reset;
RESET_REASON reason_rtc_reset[2];
WAKEUP_REASON reason_rtc_wakeup;

// --------------------------------------------------------------------------
// ESP32 Memory Map
// --------------------------------------------------------------------------
//
// +-----------------+-------------+-------------+--------+--------------------+
// | Target          |Start Address| End Address | Size   |Target              |
// +-----------------+-------------+-------------+--------+--------------------+
// | Internal SRAM 0 | 0x4007_0000 | 0x4009_FFFF | 192 kB | Instruction, Cache |
// +-----------------+-------------+-------------+--------+--------------------+
// | Internal SRAM 1 | 0x3FFE_0000 | 0x3FFF_FFFF | 128 kB | Data, DAM          |
// |                 | 0x400A_0000 | 0x400B_FFFF |        | Instruction        |
// +-----------------+-------------+-------------+--------+--------------------+
// | Internal SRAM 2 | 0x3FFA_E000 | 0x3FFD_FFFF | 200 kB | Data, DMA          |
// +-----------------+-------------+-------------+--------+---------------------+

// see also C:\Espressif\esp-idf\components\soc\esp32\soc_memory_layout.c
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/mem_alloc.html

extern "C"
void printHeapInfo( void )
{
   ESP_LOGI( TAG, "--- HeapInfo -------------------------------------------------------------" );
#if 0
   heap_caps_print_heap_info( MALLOC_CAP_8BIT );
   printf( "\n\n" );
   heap_caps_print_heap_info( MALLOC_CAP_32BIT | MALLOC_CAP_EXEC );
   printf( "\n\n" );
   heap_caps_print_heap_info( MALLOC_CAP_32BIT );
   printf( "\n\n" );
   heap_caps_print_heap_info( MALLOC_CAP_DEFAULT );
   printf( "\n\n" );
   heap_caps_print_heap_info( MALLOC_CAP_EXEC );
   printf( "\n\n" );
#endif
   // free*max is the maximum size of memory we can allocate at once
   // free*min is the minimum size of free memory
   // free*total is the sum of all free memory
   size_t free8max    = heap_caps_get_largest_free_block( MALLOC_CAP_8BIT );
   size_t free8min    = heap_caps_get_minimum_free_size( MALLOC_CAP_8BIT );
   size_t free8total  = heap_caps_get_free_size( MALLOC_CAP_8BIT );

   size_t free32max   = heap_caps_get_largest_free_block( MALLOC_CAP_32BIT | MALLOC_CAP_EXEC );
   size_t free32min   = heap_caps_get_minimum_free_size( MALLOC_CAP_32BIT | MALLOC_CAP_EXEC );
   size_t free32total = heap_caps_get_free_size( MALLOC_CAP_32BIT | MALLOC_CAP_EXEC );

   ESP_LOGI( TAG, "Free heap: %u bytes\n", xPortGetFreeHeapSize() );
   ESP_LOGI( TAG, "Free ( largest free blocks ) 8bit-capable memory : %6d bytes",   free8max );
   ESP_LOGI( TAG, "                            32bit-capable memory : %6d bytes\n", free32max );
   ESP_LOGI( TAG, "Free ( free min size )       8bit-capable memory : %6d bytes",   free8min );
   ESP_LOGI( TAG, "                            32bit-capable memory : %6d bytes\n", free32min );
   ESP_LOGI( TAG, "Free ( total free size )     8bit-capable memory : %6d bytes",   free8total );
   ESP_LOGI( TAG, "                            32bit-capable memory : %6d bytes\n", free32total );

   size_t freeDMAmax    = heap_caps_get_largest_free_block( MALLOC_CAP_DMA );
   size_t freeDMAmin    = heap_caps_get_minimum_free_size( MALLOC_CAP_DMA );
   size_t freeDMAtotal  = heap_caps_get_free_size( MALLOC_CAP_DMA );

   ESP_LOGI( TAG, "Free ( largest free blocks )  dma-capable memory : %6d bytes",     freeDMAmax );
   ESP_LOGI( TAG, "Free ( free min size )        dma-capable memory : %6d bytes",     freeDMAmin );
   ESP_LOGI( TAG, "Free ( total free size )      dma-capable memory : %6d bytes",     freeDMAtotal );

   ESP_LOGD( TAG, "task stack: %d bytes", uxTaskGetStackHighWaterMark( NULL ) );
}

// --------------------------------------------------------------------------
// get rtc reset cause string
// --------------------------------------------------------------------------

// see ...\esp-idf-v4.2\components\esp_rom\include\esp32\rom\rtc.h

extern "C"
const char* rtc_reset_reason2Str( int reason )
{
   const char* rst_msg;

   switch( reason )
   {
      case NO_MEAN:                 rst_msg =  "NO_MEAN";                break; //  0:
      case POWERON_RESET:           rst_msg =  "POWERON_RESET";          break; //  1: Vbat power on reset*/
      case SW_RESET:                rst_msg =  "SW_RESET";               break; //  3: Software reset digital core*/
      case OWDT_RESET:              rst_msg =  "OWDT_RESET";             break; //  4: Legacy watch dog reset digital core*/
      case DEEPSLEEP_RESET:         rst_msg =  "DEEPSLEEP_RESET";        break; //  5: Deep Sleep reset digital core*/
      case SDIO_RESET:              rst_msg =  "SDIO_RESET";             break; //  6: Reset by SLC module, reset digital core*/
      case TG0WDT_SYS_RESET:        rst_msg =  "TG0WDT_SYS_RESET";       break; //  7: Timer Group0 Watch dog reset digital core*/
      case TG1WDT_SYS_RESET:        rst_msg =  "TG1WDT_SYS_RESET";       break; //  8: Timer Group1 Watch dog reset digital core*/
      case RTCWDT_SYS_RESET:        rst_msg =  "RTCWDT_SYS_RESET";       break; //  9: RTC Watch dog Reset digital core*/
      case INTRUSION_RESET:         rst_msg =  "INTRUSION_RESET";        break; // 10: Instrusion tested to reset CPU*/
      case TGWDT_CPU_RESET:         rst_msg =  "TGWDT_CPU_RESET";        break; // 11: Time Group reset CPU*/
      case SW_CPU_RESET:            rst_msg =  "SW_CPU_RESET";           break; // 12: Software reset CPU*/
      case RTCWDT_CPU_RESET:        rst_msg =  "RTCWDT_CPU_RESET";       break; // 13: RTC Watch dog Reset CPU*/
      case EXT_CPU_RESET:           rst_msg =  "EXT_CPU_RESET";          break; // 14: for APP CPU, reseted by PRO CPU*/
      case RTCWDT_BROWN_OUT_RESET:  rst_msg =  "RTCWDT_BROWN_OUT_RESET"; break; // 15: Reset when the vdd voltage is not stable*/
      case RTCWDT_RTC_RESET:        rst_msg =  "RTCWDT_RTC_RESET";       break; // 16: RTC Watch dog reset digital core and rtc module*/
      default:                      rst_msg =  "unknown";
   }

   return rst_msg;
}

// --------------------------------------------------------------------------
// get reset cause string
// --------------------------------------------------------------------------

// see ...\esp-idf-v4.2\components\esp_system\include\esp_system.h

extern "C"
const char* reset_reason2Str( int reason )
{
   const char* rst_msg;

   switch( reason )
   {
      case ESP_RST_UNKNOWN:   rst_msg = "ESP_RST_UNKNOWN";   break;  //  0: Reset reason can not be determined
      case ESP_RST_POWERON:   rst_msg = "ESP_RST_POWERON";   break;  //  1: Reset due to power-on event
      case ESP_RST_EXT:       rst_msg = "ESP_RST_EXT";       break;  //  3: Reset by external pin (not applicable for ESP32)
      case ESP_RST_SW:        rst_msg = "ESP_RST_SW";        break;  //  4: Software reset via esp_restart
      case ESP_RST_PANIC:     rst_msg = "ESP_RST_PANIC";     break;  //  5: Software reset due to exception/panic
      case ESP_RST_INT_WDT:   rst_msg = "ESP_RST_INT_WDT";   break;  //  6: Reset (software or hardware) due to interrupt watchdog
      case ESP_RST_TASK_WDT:  rst_msg = "ESP_RST_TASK_WDT";  break;  //  7: Reset due to task watchdog
      case ESP_RST_WDT:       rst_msg = "ESP_RST_WDT";       break;  //  8: Reset due to other watchdogs
      case ESP_RST_DEEPSLEEP: rst_msg = "ESP_RST_DEEPSLEEP"; break;  //  9: Reset after exiting deep sleep mode
      case ESP_RST_BROWNOUT:  rst_msg = "ESP_RST_BROWNOUT";  break;  // 10: Brownout reset (software or hardware)
      case ESP_RST_SDIO:      rst_msg = "ESP_RST_SDIO";      break;  // 11: Reset over SDIO
      default:                rst_msg =  "unknown";
   }

   return rst_msg;
}

// --------------------------------------------------------------------------
// Print system information to console
// --------------------------------------------------------------------------

extern "C"
void printSystemInfo( void )
{
   /* Print chip information */
   // see C:\Espressif\frameworks\esp-idf-v5.2\components\esp_hw_support\include\esp_chip_info.h
   esp_chip_info_t chip_info;
   esp_chip_info( &chip_info );

   // build time
   char build_time_str[64];
   struct tm timeinfo;
   localtime_r( &build_utime, &timeinfo );
   strftime( build_time_str, sizeof( build_time_str ), "%c", &timeinfo );

   uint8_t mac_addr[6];
   esp_read_mac( mac_addr, ESP_MAC_WIFI_STA );

   printf( "\n\n" );
   printf( "-------------------------------------------\n" );
   printf( "BOOTUP...\n" );
   printf( "\n" );
   printf( "ESP32 platform starting...\n" );
   printf( "==== System info: ====\n" );
   printf( "IDF version:    %s\n", esp_get_idf_version() );
   printf( "Project:        ESP32 - %s%s\n", PRJ_NAME, PRJ_VARIANT );
   printf( "Build time:     %s\n", build_time_str );
   printf( "Build number:   %u\n", build_number );
   printf( "Model:          %s\n", ( chip_info.model == CHIP_ESP32       ? "ESP32"        :
                                     chip_info.model == CHIP_ESP32S2     ? "ESP32-S2"     :
                                     chip_info.model == CHIP_ESP32S3     ? "CHIP_ESP32S3" :
                                     chip_info.model == CHIP_ESP32C3     ? "CHIP_ESP32C3" :
                                     chip_info.model == CHIP_ESP32C2     ? "CHIP_ESP32C2" :
                                     chip_info.model == CHIP_ESP32C6     ? "CHIP_ESP32C6" :
                                     chip_info.model == CHIP_ESP32H2     ? "CHIP_ESP32H2" :
                                     chip_info.model == CHIP_ESP32P4     ? "CHIP_ESP32P4" :
                                     chip_info.model == CHIP_POSIX_LINUX ? "CHIP_POSIX_LINUX" : // The code is running on POSIX/Linux simulator
                                     "--" ) );
   printf( "Silicon rev.:   %d\n", chip_info.revision );
   printf( "Num cores:      %d\n", chip_info.cores );
   printf( "Features:       WiFi%s%s\n", ( chip_info.features & CHIP_FEATURE_BT ) ? "/BT"   : "",
                                         ( chip_info.features & CHIP_FEATURE_BLE ) ? "/BLE" : "" );
   printf( "CPU freq:       %d MHz\n", esp_clk_cpu_freq() / 1000 / 1000 );
   uint32_t size_flash_chip;
   esp_flash_get_size( NULL, &size_flash_chip );
   printf( "Flash size:     %dMB %s\n", size_flash_chip / ( 1024 * 1024 ),
                                        ( chip_info.features & CHIP_FEATURE_EMB_FLASH ) ? "embedded" : "external" );
   printf( "MAC address:    %02x:%02x:%02x:%02x:%02x:%02x\n",
                              ( uint32_t )mac_addr[ 0 ], ( uint32_t )mac_addr[ 1 ], ( uint32_t )mac_addr[ 2 ],
                              ( uint32_t )mac_addr[ 3 ], ( uint32_t )mac_addr[ 4 ], ( uint32_t )mac_addr[ 5 ] );
   printf( "Client name:    %s-%X\n", PRJ_NAME, *( uint32_t *) &mac_addr[ 2 ] );
   printf( "Free heap size: %u/%u/%u bytes\n", heap_caps_get_largest_free_block( MALLOC_CAP_8BIT ), xPortGetFreeHeapSize(), esp_get_minimum_free_heap_size() );
   printf( "==== End System info ====\n" );
   printf( "-------------------------------------------\n\n" );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

extern "C"
void printTaskList( void )
{
#if CONFIG_FREERTOS_USE_TRACE_FACILITY
   // see C:\espressif\esp-idf-master\examples\system\console\advanced\components\cmd_system\cmd_system.c
   const size_t bytes_per_task = 40; /* see vTaskList description */
   char* task_list_buffer = ( char* )malloc( uxTaskGetNumberOfTasks() * bytes_per_task + 128 );

   if( task_list_buffer == NULL )
   {
      ESP_LOGE( __func__, "failed to allocate buffer for vTaskList output" );
   }
   else
   {
#ifdef CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
      int length = sprintf( task_list_buffer, "\nTask Name\tStatus\tPrio\tHWM\tTask#\tCoreID\n" );
#else
      int length = sprintf( task_list_buffer, "\nTask Name\tStatus\tPrio\tHWM\tTask#\n" );
#endif
      vTaskList( task_list_buffer + length );
      ESP_LOGI( TAG, "%s\n", task_list_buffer );
      free( task_list_buffer );
   }
#endif
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

void PinsInit( void )
{
}

// --------------------------------------------------------------------------
// the main program
// --------------------------------------------------------------------------

extern "C"
void SetAppLogLevels( int app_default_log_level )
{
      esp_log_level_set( "*", app_default_log_level );      // set all components to WARN level

// --- ESP-IDF --------------------------------------------------------------
   esp_log_level_set( "cpu_start",        ESP_LOG_WARN );
   esp_log_level_set( "heap_init",        ESP_LOG_WARN );
   esp_log_level_set( "spi_flash",        ESP_LOG_WARN );
   esp_log_level_set( "gpio",             ESP_LOG_WARN );
   esp_log_level_set( "system_api",       ESP_LOG_WARN );
   esp_log_level_set( "wifi",             ESP_LOG_ERROR );
   esp_log_level_set( "wifi:wifi",        ESP_LOG_WARN );
   esp_log_level_set( "wifi:config",      ESP_LOG_WARN );
   esp_log_level_set( "wifi:Init",        ESP_LOG_WARN );
   esp_log_level_set( "wifi:Set",         ESP_LOG_WARN );
   esp_log_level_set( "wifi_init",        ESP_LOG_WARN );

// --- Arduino-------------------------------------------------------------
//    esp_log_level_set(  "ARDUINO",              ESP_LOG_WARN );

// --- ComponentsLib --------------------------------------------------------
   esp_log_level_set( "lvgl_helpers",         ESP_LOG_DEBUG );

// --- project components ---------------------------------------------------

// --- project  ---------------------------------------------------

// lvgl_tft
//    esp_log_level_set( "EPDIY",            ESP_LOG_WARN );
//    esp_log_level_set( "disp_spi",         ESP_LOG_WARN );
//    esp_log_level_set( "EPDIY",            ESP_LOG_WARN );
//    esp_log_level_set( "GC9A01",           ESP_LOG_WARN );
//    esp_log_level_set( "HX8357",           ESP_LOG_WARN );
//    esp_log_level_set( "IL3820",           ESP_LOG_WARN );
//    esp_log_level_set( "ILI9163C",         ESP_LOG_WARN );
    esp_log_level_set( "ILI9341",          ESP_LOG_DEBUG );
//    esp_log_level_set( "ILI9481",          ESP_LOG_WARN );
//    esp_log_level_set( "ILI9486",          ESP_LOG_WARN );
//    esp_log_level_set( "ILI9488",          ESP_LOG_WARN );
//    esp_log_level_set( "Main",             ESP_LOG_WARN );
//    esp_log_level_set( "lv_jd79653a",      ESP_LOG_WARN );
//    esp_log_level_set( "lv_pcd8544",       ESP_LOG_WARN );
//    esp_log_level_set( "RA8875",           ESP_LOG_WARN );
//    esp_log_level_set( "SH1107",           ESP_LOG_WARN );
//    esp_log_level_set( "SSD1306",          ESP_LOG_WARN );
//    esp_log_level_set( "ST7735S",          ESP_LOG_WARN );
//    esp_log_level_set( "st7789",           ESP_LOG_WARN );
//    esp_log_level_set( "st7789v",          ESP_LOG_WARN );
//    esp_log_level_set( "ST7796S",          ESP_LOG_WARN );
//    esp_log_level_set( "ST7796_PARALLEL",  ESP_LOG_WARN );
//    esp_log_level_set( "lv_uc8151d",       ESP_LOG_WARN );
//    esp_log_level_set( "lv_uc8176",        ESP_LOG_WARN );

// touch
//    esp_log_level_set( "ADCRAW",           ESP_LOG_WARN );
//    esp_log_level_set( "CST816T",          ESP_LOG_WARN );
//    esp_log_level_set( "FT6X36",           ESP_LOG_WARN );
//    esp_log_level_set( "GT911",            ESP_LOG_WARN );
//    esp_log_level_set( "L58",              ESP_LOG_WARN );
//    esp_log_level_set( "RA8875-Touch",     ESP_LOG_WARN );
//    esp_log_level_set( "STMPE610",         ESP_LOG_WARN );
   esp_log_level_set( "XPT2046",          ESP_LOG_INFO );

// --- project main ---------------------------------------------------------
   esp_log_level_set( "app_gui",          ESP_LOG_INFO );
   esp_log_level_set( "app_BB3",          ESP_LOG_DEBUG );
}

extern "C"
void app_main( void )
{
   SetAppLogLevels( ESP_LOG_INFO );

   esp_err_t err = ESP_OK;

   // ----------------------------------
   // get and save reset reasons
   // ----------------------------------

   reason_esp_reset = esp_reset_reason();
   reason_rtc_reset[0] = rtc_get_reset_reason( 0 );
   reason_rtc_reset[1] = rtc_get_reset_reason( 1 );
   reason_rtc_wakeup   = rtc_get_wakeup_cause();

   // ----------------------------------
   // setup gpio interrupt handler
   // ----------------------------------
   // gpio interrupt sources
   // * WPS_BTN ( buttons.c )

#if 0
   // install gpio isr service
   ESP_LOGI( TAG, "Install gpio isr service ... " );
   err = gpio_install_isr_service( /*ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 |*/ ESP_INTR_FLAG_IRAM );
   if( ESP_OK != err )
   {
      if( err != ESP_ERR_INVALID_STATE )
      {
         ESP_LOGE( TAG, "gpio_install_isr_service failed (0x%x)", err );
         // !!! todo: what to do in the case of an error
         // goto fail;
      }
      else
      {
         ESP_LOGW( TAG, "gpio_install_isr_service already installed" );
      }
   }
#endif

   // ----------------------------------
   // configure pins
   // ----------------------------------

   HEAP_INFO( "configure pins" );
   ESP_LOGI( TAG, "configure pins" );

   PinsInit();

   // ----------------------------------
   // print system info
   // ----------------------------------

   HEAP_INFO( "print system info" );
   ESP_LOGI( TAG, "print system info" );

   printSystemInfo();

   // ----------------------------------
   // init nvs flash
   // ----------------------------------

   HEAP_INFO( "init nvs flash" );
   ESP_LOGD( TAG, "Starting nvs_flash_init" );

   // initialize NV flash memory storage
   err = nvs_flash_init();
   if( err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND )
   {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK( nvs_flash_erase() );
      err = nvs_flash_init();
   }
   ESP_ERROR_CHECK( err );
#if 0
   // ----------------------------------
   // get user_settings partition for configuration
   // ----------------------------------

   HEAP_INFO( "get user_settings partition for configuration" );
   ESP_LOGI( TAG, "get user_settings partition for configuration" );

#define  USER_SETTINGS_PARTITION_TYPE      ESP_PARTITION_TYPE_DATA
#define  USER_SETTINGS_PARTITION_SUBTYPE   ESP_PARTITION_SUBTYPE_DATA_FAT
#define  USER_SETTINGS_PARTITION_NAME      "user_settings"
#define  USER_SETTINGS_PARTITION_INFO_FLAG 12138     // ??? not used yet

   uint32_t configs_start_addr = 0;
   uint32_t configs_size = 0;

   const esp_partition_t *pt = esp_partition_find_first( USER_SETTINGS_PARTITION_TYPE,
                                                         USER_SETTINGS_PARTITION_SUBTYPE,
                                                         USER_SETTINGS_PARTITION_NAME );
   if( pt == NULL )
   {
      ESP_LOGE( TAG, "Partion \"%s\" not found", _s( USER_SETTINGS_PARTITION_NAME ) );
   }
   else
   {
      configs_start_addr = pt->address;
      configs_size = pt->size;

      ESP_LOGI( TAG, "Name  \"%s\" start 0x%08x size %d", _s( pt->label ), configs_start_addr, configs_size );
   }
#endif
   // ----------------------------------
   // Start the Display and Touch Task
   // ----------------------------------

   HEAP_INFO( "start gui task (display and touch)" );
   ESP_LOGI( TAG, "start gui task (display and touch) ..." );

   // show welcome screen
   appGui();
   appBB3();

  // ----------------------------------
   // system and application setup done
   // ----------------------------------

   HEAP_INFO( "system and application setup done" );
   ESP_LOGI( TAG, "system and application setup done" );
   ESP_LOGI( TAG, "--------------------------------------------------------------------------" );

   printHeapInfo();
   printTaskList();

   HEAP_INFO( "Ready ..." );
   ESP_LOGI( TAG, "Ready ..." );
   ESP_LOGI( TAG, "--------------------------------------------------------------------------\n\n" );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
