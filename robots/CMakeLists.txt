# Suppress warnings for all mbed-os sources
file(GLOB_RECURSE MBED_C_SOURCES "${CMAKE_SOURCE_DIR}/mbed-os/*.c")
file(GLOB_RECURSE MBED_CXX_SOURCES "${CMAKE_SOURCE_DIR}/mbed-os/*.cpp")

set_source_files_properties(
    ${MBED_C_SOURCES}
    ${MBED_CXX_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)


add_library(robots_common)

target_include_directories(robots_common PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})



add_subdirectory(Infantry EXCLUDE_FROM_ALL)
# TODO FIX ME
# add_subdirectory(TestBench EXCLUDE_FROM_ALL)
# add_subdirectory(Engineer EXCLUDE_FROM_ALL)
# add_subdirectory(Sentry EXCLUDE_FROM_ALL)
# add_subdirectory(Hero EXCLUDE_FROM_ALL)
