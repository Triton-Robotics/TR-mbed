project(TR-core)

add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/subsystems
        ${CMAKE_CURRENT_SOURCE_DIR}/util
)

add_subdirectory(util)
add_subdirectory(subsystems)