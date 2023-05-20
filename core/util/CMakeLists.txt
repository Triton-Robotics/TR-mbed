file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp *.h *.hpp)

target_sources(${PROJECT_NAME} PUBLIC ${SOURCES})


target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/util
        )

add_subdirectory(algorithms)
add_subdirectory(communications)
add_subdirectory(peripherals)
add_subdirectory(motor)