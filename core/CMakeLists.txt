project(TR-core)

add_library(${PROJECT_NAME} subsystems/ChassisKalman.cpp subsystems/ChassisKalman.h)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/subsystems
        ${CMAKE_CURRENT_SOURCE_DIR}/util
        ${CMAKE_SOURCE_DIR}/libs/tinyekf/src
)

message("source dir: ${CMAKE_SOURCE_DIR}")

target_sources(${PROJECT_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/libs/tinyekf/src/tiny_ekf.h
        ${CMAKE_SOURCE_DIR}/libs/tinyekf/src/tiny_ekf.c
        ${CMAKE_SOURCE_DIR}/libs/tinyekf/src/TinyEKF.h
        ${CMAKE_SOURCE_DIR}/libs/tinyekf/src/tiny_ekf_struct.h
)

add_subdirectory(util)
add_subdirectory(subsystems)