project(TR-algorithms)

set(HEADER_FILES
        PID.h
        speedtocurrent.hpp
)

set(SRC_FILES
        PID.cpp
)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SRC_FILES})