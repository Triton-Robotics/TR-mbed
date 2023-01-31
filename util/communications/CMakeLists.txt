set(HEADER_FILES
        CANMsg.h
        DJIRemote.h
        djiremoteuart.h
        SerialCommunication.h
        serialLogger.h
        newCANHandler.h
)

set(SRC_FILES
        DJIRemote.cpp
)

target_sources(${PROJECT_NAME} PUBLIC ${HEADER_FILES} ${SRC_FILES})

add_subdirectory(referee)