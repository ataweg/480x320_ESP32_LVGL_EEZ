# 2021-10-10 AWe  add creation of build_info header file

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required( VERSION 3.5 )

set( PRJ_NAME "480x320_ESP32_LVGL_EEZ" )
set( PRJ_VARIANT "" )
set( FIRMWARE_NAME "480x320_ESP32_LVGL_EEZ" )

# declare for some components options
set( CONFIGS_USE_LIBESPHTTPD "y" )
set( LIBESPHTTPD_USE_ESPFS "y" )

message( "Build direcroy: ${BUILD_DIR}" )
message( "Prj name: ${PRJ_NAME}" )
message( "Prj variant: ${PRJ_VARIANT}" )
message( "Prj directory: ${CMAKE_SOURCE_DIR}" )
message( "Build directory: ${MY_BUILD_DIR}" )

list( APPEND EXTRA_COMPONENT_DIRS
   "F:/Projects/InternetOfThings/ComponentsLib/sources"
)

# shorten component list
set( COMPONENTS
   "main"               # required
   "espcoredump"
)

include( $ENV{IDF_PATH}/tools/cmake/project.cmake )
project( "${FIRMWARE_NAME}" )
message( "Firmware name: ${FIRMWARE_NAME}" )

list( APPEND compile_options
   "-DESP32"
   -DFREERTOS
   -DPRJ_NAME="${PRJ_NAME}"
   -DPRJ_VARIANT="${PRJ_VARIANT}"
   "-DLV_CONF_PATH=${CMAKE_SOURCE_DIR}/main/lv_conf.h"
   -DEEZ_FOR_LVGL
)

list( APPEND compile_options
   "-Wno-error=format="
   "-Wno-format"
   "-Wno-unused-variable"
   "-Wno-ignored-qualifiers"
#   "-Wno-error=unused-const-variable"
#   "-Wno-unused-const-variable"
#   "-Wno-error=implicit-function-declaration"
#   "-Wno-implicit-fallthrough"
#   "-Wno-error=tautological-compare"
#   "-Wno-tautological-compare"
#   "-Wno-error=address"
#   "-Wno-unused-function"
)

## for use with gcc 8.20
#list( APPEND c_compile_options
#   "-Wno-incompatible-pointer-types"
#   "-Wno-address-of-packed-member"
#)

list( APPEND cxx_compile_options
   "-Wno-missing-field-initializers"
   "-Wno-error=missing-field-initializers"
#   "-Wno-deprecated-enum-enum-conversion"
#   "-Wno-error=overloaded-virtual"
)

idf_build_set_property( COMPILE_OPTIONS     "${compile_options}"     APPEND )
idf_build_set_property( C_COMPILE_OPTIONS   "${c_compile_options}"   APPEND )
idf_build_set_property( CXX_COMPILE_OPTIONS "${cxx_compile_options}" APPEND )
idf_build_set_property( COMPILE_DEFINITIONS "${compile_definitions}" APPEND )
idf_build_set_property( LINK_OPTIONS        "${link_options}"        APPEND )

add_compile_options( -fdiagnostics-color=always )

# see https://stackoverflow.com/questions/24292898/compile-date-and-time-using-cmake
#     https://stackoverflow.com/questions/35745344/cmake-target-version-increment
# https://cmake.cmake.narkive.com/GxXu0WUq/howto-increment-build-number-with-cmake
# https://stackoverflow.com/questions/42274437/how-to-have-an-auto-incrementing-build-version-number-kdevelop

add_custom_target(
   buildnumber
   COMMAND ${CMAKE_COMMAND}
          -DHEADER_DIR:PATH="${CMAKE_SOURCE_DIR}/main/"
          -DBUILD_NUMBER_DIR:PATH="${PROJECT_DIR}"
          -DPROJECT_DIR:PATH="${PROJECT_DIR}"
          -P "${PROJECT_DIR}/buildnumber.cmake"
)

add_dependencies( ${project_elf} buildnumber )
