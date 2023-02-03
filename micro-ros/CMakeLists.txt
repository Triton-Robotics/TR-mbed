# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

set(APP_TARGET TR-jetson)

add_library(${APP_TARGET})

include(ExternalProject)

externalproject_add(libmicroros_project
        PREFIX     ${CMAKE_BINARY_DIR}/libmicroros-prefix
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND
        make -f ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros.mk
        X_CC=${CMAKE_C_COMPILER}
        X_AR=${CMAKE_AR}
        X_STRIP=${CMAKE_STRIP}
        X_CFLAGS=${CMAKE_C_FLAGS}
        X_CXX=${CMAKE_CXX_COMPILER}
        X_CXXFLAGS=${CMAKE_CXX_FLAGS}
        BUILD_DIR=${CMAKE_BINARY_DIR}
        EXTENSIONS_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        INSTALL_COMMAND ""
        BUILD_BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros.a
        )

project(${APP_TARGET})

target_sources(${APP_TARGET}
        PUBLIC
        jetson.h
        jetson.cpp
)

add_dependencies(${APP_TARGET} libmicroros_project)

target_include_directories(${APP_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${APP_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

execute_process(
        COMMAND
        make -f ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros.mk get_package_names
        WORKING_DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE
        INCLUDE_ROS2_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

foreach(pkg ${INCLUDE_ROS2_PACKAGES})
    target_include_directories(${APP_TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/${pkg})
endforeach()


option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
