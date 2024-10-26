file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp *.h *.hpp)

target_sources(${PROJECT_NAME} PUBLIC ${SOURCES})

add_subdirectory(examples EXCLUDE_FROM_ALL)

target_include_directories(TR-imu-examples PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})