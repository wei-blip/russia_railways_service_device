cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED BOARD)
    set(BOARD rs_0015_100)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../modules/rts-zephyr/clion-zephyr.cmake)

project(service_device VERSION 0.0.0)
FILE(GLOB_RECURSE SOURCES src/*.c)
target_sources(app PRIVATE ${SOURCES})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/libs)
