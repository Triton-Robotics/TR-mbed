project(TR-util)

add_library(${PROJECT_NAME} helperFunctions.hpp buttonanalyzer.hpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(algorithms)
add_subdirectory(communications)
add_subdirectory(peripherals)
add_subdirectory(motor)
add_subdirectory(subsystems)
