project(TR-imu)

add_executable(${PROJECT_NAME}
        BNO055.h
        BNO055.cpp
        imuExampleAllData.cpp
        imuExampleQuaternions.cpp
)