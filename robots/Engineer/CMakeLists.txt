project(TR-Engineer)

add_executable(${PROJECT_NAME})

mbed_configure_app_target(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/util)

mbed_set_post_build(${PROJECT_NAME})