set(HEADER_FILES
        PWMMotor.h
        DJIMotor.h
)

set(SRC_FILES
        PWMMotor.cpp
        DJIMotor.cpp
)

target_sources(${PROJECT_NAME} PUBLIC ${HEADER_FILES} ${SRC_FILES})