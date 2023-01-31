set(HEADER_FILES
        PID.h
        speedtocurrent.hpp
)

set(SRC_FILES
        PID.cpp
)

target_sources(${PROJECT_NAME} PUBLIC ${HEADER_FILES} ${SRC_FILES})