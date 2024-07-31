cmake_minimum_required( VERSION 3.5 )

file(GLOB_RECURSE COMPONENT_SRCS
   *.c
   *.cpp
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DEEZ_PLATFORM_ESP32 -DEEZ_FOR_LVGL")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEEZ_PLATFORM_ESP32 -DEEZ_FOR_LVGL")

set( COMPONENT_INCLUDE_DIRS
   "."
   "./include"
)

set( PRIV_COMPONENT_REQUIRES
   "nvs_flash"
   "driver"
)

set( COMPONENT_REQUIRES
   "dbg_helper"                  # dbg_mem.h
   "arduino"
   "lvgl"                        # lvgl.h
   "TFT_eSPI"
   "eez-framework"
   "esp_netif"
   "esp_wifi"
)

idf_component_register(
   SRCS              "${COMPONENT_SRCS}"
   INCLUDE_DIRS      "${COMPONENT_INCLUDE_DIRS}"
   PRIV_REQUIRES     "${PRIV_COMPONENT_REQUIRES}"
   REQUIRES          "${COMPONENT_REQUIRES}"
)

message( "Main Project: ${PROJECT_NAME}" )
