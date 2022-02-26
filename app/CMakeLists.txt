cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED BOARD)
    set(BOARD nucleo_l452re)
endif()
#set(BOARD decawave_dwm1001_dev)

include(${CMAKE_CURRENT_SOURCE_DIR}/../modules/rts-zephyr/clion-zephyr.cmake)

project(lora_russia_railways VERSION 0.0.0)
FILE(GLOB_RECURSE SOURCES src/*.c)
target_sources(app PRIVATE ${SOURCES})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/libs)