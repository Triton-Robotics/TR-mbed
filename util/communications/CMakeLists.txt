set(HEADER_FILES
        CANMsg.h
        DJIRemote.h
        crc.h
        ref_serial.h
        ref_ui.h
        djiremoteuart.h
        SerialCommunication.h
        serialLogger.h
        newCANHandler.h
)

set(SRC_FILES
        DJIRemote.cpp
        crc.cpp
        ref_serial.cpp
        ref_ui.cpp
)

target_sources(${PROJECT_NAME} PUBLIC ${HEADER_FILES} ${SRC_FILES})