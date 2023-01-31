project(TR-communications)

add_executable(${PROJECT_NAME}
        include/DJIRemote.hpp
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