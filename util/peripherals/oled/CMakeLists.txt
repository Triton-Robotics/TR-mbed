project(TR-oled)

set(HEADER_FILES
        font_8x8.h
        font_16x24.h
        mbed_logo.h
        SSD1308.h
)

set(SRC_FILES
        SSD1308.cpp
)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SRC_FILES})