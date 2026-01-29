# Suppress warnings for all mbed-os sources
file(GLOB_RECURSE MBED_C_SOURCES "${CMAKE_SOURCE_DIR}/mbed-os/*.c")
file(GLOB_RECURSE MBED_CXX_SOURCES "${CMAKE_SOURCE_DIR}/mbed-os/*.cpp")

set_source_files_properties(
    ${MBED_C_SOURCES}
    ${MBED_CXX_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)

add_library(TR-core)

target_include_directories(TR-core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

target_sources(TR-core PRIVATE ${SOURCES})
