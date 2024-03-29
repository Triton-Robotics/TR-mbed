cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

find_program(MBED_CLI mbed-tools PATHS "~/.local/bin" REQUIRED)

# User variables, feel free to change as necessary
set(MBED_PLATFORM NUCLEO_F446RE)
set(MBED_TOOLCHAIN GCC_ARM)
set(DISABLE_WARNINGS ON)

# Check if mbed-os is downloaded, if not download it
if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os)
    message("Beginning mbed-os download...")
    execute_process(
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMAND ${MBED_CLI} deploy
    )
    message("Finished!")
endif ()


# Setup CMake for the target device
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/mbed_config.cmake)
    message("Configuring mbed-os for ${MBED_PLATFORM} with ${MBED_TOOLCHAIN}...")

    execute_process(
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMAND ${MBED_CLI} configure -m ${MBED_PLATFORM} -t ${MBED_TOOLCHAIN}
    )

    # Manually move mbed config output to user specified build directory
    # ??? This should be a CLI option for mbed-tools, smh
    execute_process(
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMAND mv cmake_build/${MBED_PLATFORM}/develop/${MBED_TOOLCHAIN}/mbed_config.cmake ${CMAKE_CURRENT_BINARY_DIR}
            COMMAND mv cmake_build/${MBED_PLATFORM}/develop/${MBED_TOOLCHAIN}/.mbedignore ${CMAKE_CURRENT_BINARY_DIR}
    )
    # Remove old build directory that mbed generates
    file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/cmake_build)
    message("Finished!")
endif ()


# mbed variables
set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET TR-Embedded)

if (DISABLE_WARNINGS)
    add_compile_options(-w)
endif()

# Enable cmake for mbed
include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH})
link_libraries(mbed-os)

add_subdirectory(core)
link_libraries(TR-core)

add_subdirectory(robots)

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()