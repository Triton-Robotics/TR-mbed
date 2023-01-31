set(HEADER_FILES
        ref_serial.h
        ref_ui.h
        crc.h
)

set(SRC_FILES
        ref_serial.cpp
        ref_ui.cpp
        crc.cpp
)

target_sources(${PROJECT_NAME} PUBLIC ${HEADER_FILES} ${SRC_FILES})