set(HEADER_FILES
        PID.h
        speedtocurrent.hpp
        TRMutex.h
)

set(SRC_FILES
        PID.cpp
        TRMutex.cpp
)

target_sources(${PROJECT_NAME} PUBLIC ${HEADER_FILES} ${SRC_FILES})