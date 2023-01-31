project(TR-Sentry)

add_executable(${PROJECT_NAME})

mbed_configure_app_target(${PROJECT_NAME})

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
target_sources(${PROJECT_NAME} PRIVATE main.cpp ${PARENT_DIR}/main.h)

mbed_set_post_build(${PROJECT_NAME})