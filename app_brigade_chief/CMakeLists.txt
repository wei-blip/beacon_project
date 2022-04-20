cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED BOARD)
    set(BOARD nucleo_l452re)
#    set(BOARD rs_0015_100)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../modules/rts-zephyr/clion-zephyr.cmake)

project(brigade_chief_russia_railways VERSION 0.0.0)
FILE(GLOB_RECURSE SOURCES src/*.c src/*.cc src/*.cpp)
target_sources(app PRIVATE ${SOURCES})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/libs)
