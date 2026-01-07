project(TR-core)
add_library(${PROJECT_NAME}
        util/algorithms/PIDTuner.cpp
        util/algorithms/PIDTuner.h
        util/algorithms/PIDTools.cpp
        util/algorithms/PIDTools.h)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(util)
add_subdirectory(subsystems)
add_subdirectory(commands)