if(ESP_PLATFORM)
file(GLOB_RECURSE COMPONENT_SRCS
   "./src/*.c"
   "./src/*.cpp"
)

set( COMPONENT_INCLUDE_DIRS
   "./src"
   "./src/eez/libs/agg"
   "./src/eez/platform/simulator"
)

set( COMPONENT_REQUIRES
    lvgl
)

idf_component_register(
   SRCS              "${COMPONENT_SRCS}"
   INCLUDE_DIRS      "${COMPONENT_INCLUDE_DIRS}"
   PRIV_REQUIRES     "${PRIV_COMPONENT_REQUIRES}"
   REQUIRES          "${COMPONENT_REQUIRES}"
)
else()
    cmake_minimum_required(VERSION 3.12)

    include_directories(
        ./src
        ./src/eez/libs/agg
    )

    file(GLOB_RECURSE SOURCES
        ./src/eez/*.cpp
        ./src/eez/*.c
    )

    ADD_LIBRARY(eez-framework STATIC ${SOURCES})

    target_include_directories(eez-framework SYSTEM PUBLIC ./src ./src/eez/libs/agg)
endif()
