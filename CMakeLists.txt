# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

# User variables, feel free to change as necessary
set(USE_MICRO_ROS ON)

set(DISABLE_WARNINGS ON)
if (DISABLE_WARNINGS)
    add_compile_options(-w)
endif()

# Setup CMake for the target device
execute_process(
        COMMAND mbed-tools configure -m NUCLEO_F446RE -t GCC_ARM
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# mbed variables
set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET TR-Embedded)

# Enable cmake for mbed
include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH})

link_libraries(mbed-os)

if (USE_MICRO_ROS)
    add_subdirectory(micro-ros)
    link_libraries(TR-jetson)
endif()

add_subdirectory(core)
link_libraries(TR-core)

add_subdirectory(robots)


option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
