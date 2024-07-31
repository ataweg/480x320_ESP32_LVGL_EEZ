// --------------------------------------------------------------------------
//
// Project       Wifi-ControlUnit
//
// File          app_gui.cpp
//
// Author        Axel Werner
//
// --------------------------------------------------------------------------
// Changelog
//
// 2023-02-21  AWe   do the setup function before the task is created
// 2019-10-01  AWe   initial version
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// debug support
// --------------------------------------------------------------------------

static const char *TAG = "app_gui";
#include "esp_log.h"

// #define DBG_MEMLEAK          // confict wit eez-framwork
#define DBG_HEAP_INFO

// #define NO_DBG_MEMLEAK
// #define NO_DBG_HEAP_INFO

#include "dbg_mem.h"
#include "dbg_helper.h"          // HEAP_INFO()

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

// RTOS Include
#include "freertos/FreeRTOS.h"      // xPortGetFreeHeapSize()
#include "freertos/task.h"          // vTaskDelay()
#include "freertos/semphr.h"

#include "esp_timer.h"

// Littlevgl specific
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
   #include "lvgl.h"               // LV_HOR_RES_MAX, LV_VER_RES_MAX
#else
   #include "lvgl/lvgl.h"          // LVGL header file
#endif

#include "TFT_eSPI.h"
#include "ui/ui.h"

// Include desired font here for espressif/esp-iot-solution example

// LV_IMG_DECLARE(mouse_cursor_icon);         /* Declare the image file. */

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define LV_TICK_PERIOD_MS            10

#define LINES_TO_DRAW               24
#define USE_STATIC_DISPLAY_BUFFER    0      // using static buffers will result in a crash
#define USE_DOUBLE_DISPLAY_BUFFER    0      // no double buffer saves 20KB memory

// --------------------------------------------------------------------------
// set stack size
// --------------------------------------------------------------------------

#ifndef CONFIG_TASK_STACK_SIZE_GUI
   #define TASK_STACK_SIZE_GUI            ( 6 * 1024 )
#else
   #define TASK_STACK_SIZE_GUI            CONFIG_TASK_STACK_SIZE_GUI
#endif

#ifndef CONFIG_TASK_CORE_GUI
   #define CONFIG_TASK_CORE_GUI        tskNO_AFFINITY
#endif
#ifndef CONFIG_TASK_PRIO_GUI
   #define CONFIG_TASK_PRIO_GUI        6
#endif

// --------------------------------------------------------------------------
// local functions
// --------------------------------------------------------------------------

static void lv_tick_task( void *arg );
static void appGuiTask( void *pvParameter );

// --------------------------------------------------------------------------
//  typedefs
// --------------------------------------------------------------------------

typedef struct
{
   TFT_eSPI * tft;
} lv_tft_espi_t;

// --------------------------------------------------------------------------
//  local variables
// --------------------------------------------------------------------------

DMA_ATTR uint8_t* disp_buf1 =  NULL;
DMA_ATTR uint8_t* disp_buf2 =  NULL;

// --------------------------------------------------------------------------
// Serial debugging
// --------------------------------------------------------------------------

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
   printf( "%s", buf );
   // flush();
}
#endif

// --------------------------------------------------------------------------
// tft display flushing
// --------------------------------------------------------------------------

void tft_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map )
{
   lv_tft_espi_t * dsc = ( lv_tft_espi_t * )lv_display_get_driver_data( disp );

   uint32_t w = lv_area_get_width( area );
   uint32_t h = lv_area_get_height( area );

   dsc->tft->startWrite();
   dsc->tft->setAddrWindow( area->x1, area->y1, w, h );
   dsc->tft->pushColors( ( uint16_t * )px_map, w * h, true );
   dsc->tft->endWrite();
}

// --------------------------------------------------------------------------
// gui flushing
// --------------------------------------------------------------------------

void my_display_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map )
{
   tft_disp_flush( disp, area, px_map );
   lv_display_flush_ready( disp );
}

// --------------------------------------------------------------------------
// Read the touchpad
// --------------------------------------------------------------------------

bool tft_touchpad_read( lv_indev_t * indev_driver, lv_indev_data_t * data )
{
   uint16_t touchX, touchY;

   lv_tft_espi_t * dsc = ( lv_tft_espi_t * )lv_indev_get_driver_data( indev_driver );

   bool touched = dsc->tft->getTouch( &touchX, &touchY, 600 );

   if( !touched )
   {
      data->state = LV_INDEV_STATE_REL;
   }
   else
   {
      data->state = LV_INDEV_STATE_PR;

      /* Set the coordinates */
      data->point.x = touchX;
      data->point.y = touchY;

      ESP_LOGD( TAG, "Data x %3d, y %3d", touchX, touchY );
   }

   return touched;
}

void my_touchpad_read( lv_indev_t * indev_driver, lv_indev_data_t * data )
{
   tft_touchpad_read( indev_driver, data );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
/*
   see also F:\Projects\InternetOfThings\Devices\ESP32-2432s028\Firmware\LVGL_Full_Test\source\components\lv_examples\examples\porting\lv_port_disp_template.c
      lv_port_disp_init()
*/

// Interface and driver initialization
lv_display_t *my_display_create( uint32_t hor_res, uint32_t ver_res )
{
   ESP_LOGI( TAG, "Display hor size: %d, ver size: %d", hor_res, ver_res );

   lv_tft_espi_t * dsc = ( lv_tft_espi_t * )lv_malloc_zeroed( sizeof( lv_tft_espi_t ) );
   LV_ASSERT_MALLOC( dsc );
   if( dsc == NULL )
      return NULL;

   // Create the display in LVGL. Set also the resolution of the display
   lv_display_t * disp = lv_display_create( hor_res, ver_res );
   if( disp == NULL )
   {
      lv_free( dsc );
      return NULL;
   }

   dsc->tft = new TFT_eSPI( hor_res, ver_res );
   dsc->tft->begin();          /* TFT init */
   dsc->tft->setRotation( 3 ); /* Landscape orientation, flipped */

   lv_display_set_driver_data( disp, ( void * )dsc );
   lv_display_set_flush_cb( disp, my_display_flush );

   // allocate memory for display buffers
   uint32_t buf_size_bytes = hor_res * LINES_TO_DRAW * ( ( LV_COLOR_DEPTH + 7 ) / 8 );
   ESP_LOGI( TAG, "Display buffer size: %d", buf_size_bytes );

   HEAP_INFO( "appGuiTask allocate display buffers" );
#if USE_STATIC_DISPLAY_BUFFER
   // !!! this will result in a crash
   uint8_t disp_buf1[ buf_size_bytes ];
 #if USE_DOUBLE_DISPLAY_BUFFER
   uint8_t disp_buf2[ buf_size_bytes ];
 #else
   uint8_t * disp_buf2 = NULL;
 #endif
#else
   disp_buf1 = ( uint8_t* )heap_caps_malloc( buf_size_bytes, MALLOC_CAP_DMA );
   LV_ASSERT_MSG( disp_buf1 != NULL, "Can't allocate display buffer 1" );
 #if USE_DOUBLE_DISPLAY_BUFFER
   // Use double buffered
   disp_buf2 = ( uint8_t* )heap_caps_malloc( buf_size_bytes, MALLOC_CAP_DMA );
   LV_ASSERT_MSG( disp_buf2 != NULL, "Can't allocate display buffer 2" );
 #else
   uint8_t* disp_buf2 = NULL;
 #endif
#endif
   HEAP_INFO( "appGuiTask allocate display buffers done" );
   ESP_LOGI( TAG, "Buffer addresses: 0x%08x, 0x%08x, size %d bytes ", ( void *)disp_buf1, (void *)disp_buf2, buf_size_bytes );
   lv_display_set_buffers( disp, (void *)disp_buf1, (void *)disp_buf2, buf_size_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL );

   /*Set the touchscreen calibration data,
     the actual data for your display can be aquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
   uint16_t calData[5] = { 275, 3620, 264, 3532, 1 };
   dsc->tft->setTouch( calData );

   return disp;
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

static void lv_tick_task( void *arg )
{
   ( void ) arg;
   lv_tick_inc( LV_TICK_PERIOD_MS );
}

// --------------------------------------------------------------------------
//  the Driver Monitors gui task
// --------------------------------------------------------------------------

// taken fom ...\lv_port_esp32\source\main\main.c

// Creates a semaphore to handle concurrent call to lvgl stuff
// If you wish to call *any* lvgl function from other threads/tasks
// you should lock on the very same semaphore!
SemaphoreHandle_t xGuiSemaphore;         // Create a GUI semaphore

static void appGuiTask( void *pvParameter )
{
   ESP_LOGI( TAG, "Start GUI Task" );
   ( void ) pvParameter;
   xGuiSemaphore = xSemaphoreCreateMutex();   // Create a GUI semaphore

   // Initialize LittlevGL
   lv_init();

   // Create a display and set a flush_cb
   ESP_LOGI( TAG, "Display initialization" );
   lv_display_t * disp = my_display_create( LV_HOR_RES_MAX, LV_VER_RES_MAX );

   ESP_LOGI( TAG, "setup Touch controller" );

   // Register an input device when enabled on the menuconfig
   lv_indev_t * indev_touchpad = lv_indev_create();
   lv_indev_set_type( indev_touchpad, LV_INDEV_TYPE_POINTER ); // Touchpad should have POINTER type
   // after calibration the callback function is replaced in lv_tc_indev_init()

   lv_indev_set_read_cb( indev_touchpad, my_touchpad_read );
   lv_indev_set_driver_data( indev_touchpad, ( void * )lv_display_get_driver_data( disp ) );

   // Create and start a periodic timer interrupt to call lv_tick_inc
   const esp_timer_create_args_t periodic_timer_args =
   {
      .callback = &lv_tick_task,
      .name = "periodic_gui"
   };
   esp_timer_handle_t periodic_timer;
   ESP_ERROR_CHECK( esp_timer_create( &periodic_timer_args, &periodic_timer ) );
   ESP_ERROR_CHECK( esp_timer_start_periodic( periodic_timer, LV_TICK_PERIOD_MS * 1000 ) );

//   app_backlight_setup( LCD_BL_Pin );

   /* Create the gui application */
   ESP_LOGI( TAG, "Create the gui application" );
   ui_init();

   // --------------------------------------------------------------------------
   // the task loop
   // --------------------------------------------------------------------------

   ESP_LOGD( TAG, "Run gui task ..." );

   while( true )
   {
      vTaskDelay( 1 ); // give other tasks a chance to run

      // Try to take the semaphore, call lvgl related function on success
      if( pdTRUE == xSemaphoreTake( xGuiSemaphore, portMAX_DELAY ) )
      {
         ui_tick();
         lv_timer_handler(); // let the GUI do its work
         xSemaphoreGive( xGuiSemaphore );
      }

//      app_backlight_loop();
   }

   // A task should NEVER return
   free(disp_buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
   free(disp_buf2);
#endif
   vTaskDelete( NULL );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

extern "C"
void appGui( void )
{
   ESP_LOGI( TAG, "appGuiTask" );

   // gui task init
   xTaskCreatePinnedToCore( appGuiTask, "appGuiTask",  TASK_STACK_SIZE_GUI, NULL, CONFIG_TASK_PRIO_GUI, NULL, CONFIG_TASK_CORE_GUI );
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------
