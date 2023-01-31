target_sources(${PROJECT_NAME} PUBLIC
        BNO055.h
        BNO055.cpp
)

add_subdirectory(examples EXCLUDE_FROM_ALL)

target_include_directories(TR-imu-examples PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})