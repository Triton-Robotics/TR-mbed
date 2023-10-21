project(TR-imu-examples)

add_executable(${PROJECT_NAME} imuExampleAllData.cpp imuExampleQuaternions.cpp
        ../../oled/oledDisplay.cpp
        ../../oled/oledDisplay.h)