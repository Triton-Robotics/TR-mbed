#include "mbed.h"
#include "util/peripherals/Sensor.h"

class IMU: public Sensor
{
public:
    struct EulerAngles
    {
        float yaw;
        float pitch;
        float roll;
    };

    struct Quaternion
    {
        float w; // Scalar
        float x;
        float y;
        float z;
    };

    void init();
    void read();
    void reset();
};