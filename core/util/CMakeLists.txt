file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp *.h *.hpp)

target_sources(${PROJECT_NAME} PUBLIC ${SOURCES})

add_subdirectory(algorithms)
add_subdirectory(communications)
add_subdirectory(peripherals)
add_subdirectory(motor)