# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

# User variables, feel free to change as necessary
set(USE_MICRO_ROS OFF)

set(DISABLE_WARNINGS ON)
if (DISABLE_WARNINGS)
    add_compile_options(-w)
endif()

# mbed variables
set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET TR-Embedded)

# Enable cmake for mbed
include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH})

link_libraries(mbed-os)

add_subdirectory(util)
link_libraries(TR-util)

if (USE_MICRO_ROS)
    add_subdirectory(micro-ros)
endif()

add_subdirectory(robots)


option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
