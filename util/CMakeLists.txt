project(TR-util)

add_executable(${PROJECT_NAME} helperFunctions.hpp buttonanalyzer.hpp)

add_subdirectory(algorithms)
add_subdirectory(communications)
add_subdirectory(peripherals)
add_subdirectory(motor)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})