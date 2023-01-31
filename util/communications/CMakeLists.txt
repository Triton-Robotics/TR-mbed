project(TR-communications)

add_executable(${PROJECT_NAME}
        include/DJIRemote.hpp
        oled/font_8x8.h
        oled/font_16x24.h
        oled/mbed_logo.hpp
        oled/SSD1308.hpp
        oled/SSD1308.cpp
        CANMsg.h
        crc.hpp
        crc.cpp
        DJIRemote.cpp
        djiremoteuart.hpp
        newCANHandler.hpp
        ref_serial.hpp
        ref_serial.cpp
        ref_ui.hpp
        ref_ui.cpp
        SerialCommunication.hpp
        serialLogger.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)