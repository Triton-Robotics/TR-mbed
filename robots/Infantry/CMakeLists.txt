# Suppress warnings for all mbed-os sources
file(GLOB_RECURSE MBED_C_SOURCES "${CMAKE_SOURCE_DIR}/mbed-os/*.c")
file(GLOB_RECURSE MBED_CXX_SOURCES "${CMAKE_SOURCE_DIR}/mbed-os/*.cpp")

set_source_files_properties(
    ${MBED_C_SOURCES}
    ${MBED_CXX_SOURCES}
    PROPERTIES
    COMPILE_FLAGS "-w"
)


project(TR-Infantry)

mbed_configure_app_target(${PROJECT_NAME})

add_executable(${PROJECT_NAME}
    Infantry.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE robots_common)


# TODO FIX this cmake file 

# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


# get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

# file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp *.h *.hpp)
# target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${PARENT_DIR}/main.h)

mbed_set_post_build(${PROJECT_NAME})