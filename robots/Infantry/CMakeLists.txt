project(TR-Infantry)

mbed_configure_app_target(${PROJECT_NAME})

add_executable(${PROJECT_NAME}
    # main.cpp
    Infantry.cpp
    RobotConfigInfantry.cpp
    RobotStateInfantry.cpp
)
target_link_libraries(${PROJECT_NAME} PRIVATE robots_common)

# TODO this doesnt work
target_compile_options(${PROJECT_NAME} PRIVATE 
    -Wall
    -Wextra
    -Wpedantic
    -Wreturn-type
    -Wunused
    # -Werror
)

# TODO FIX this cmake file 

# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


# get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

# file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp *.h *.hpp)
# target_sources(${PROJECT_NAME} PRIVATE ${SOURCES} ${PARENT_DIR}/main.h)

mbed_set_post_build(${PROJECT_NAME})