project(TR-Infantry)

add_executable(${PROJECT_NAME})

mbed_configure_app_target(${PROJECT_NAME})

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp *.h *.hpp)
target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${PARENT_DIR}/main.h)

mbed_set_post_build(${PROJECT_NAME})