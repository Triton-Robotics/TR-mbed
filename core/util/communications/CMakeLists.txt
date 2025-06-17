file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp *.h *.hpp)

target_sources(${PROJECT_NAME} PUBLIC ${SOURCES})

add_subdirectory(referee)
add_subdirectory(jetson)