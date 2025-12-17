add_library(TR-core)

target_include_directories(TR-core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

target_sources(TR-core PRIVATE ${SOURCES})
