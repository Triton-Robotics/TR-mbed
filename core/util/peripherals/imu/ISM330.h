/**
* @file ISM330.h
* @brief SPI driver for the ISM330DHCX 6-DoF accelerometer and gyroscope.
*
* This class provides a lightweight interface for communicating with the
* ST ISM330DHCX sensor over SPI using Mbed OS. It supports basic device
* initialization and simple readout of accelerometer and gyroscope data.
*
* The sensor must be externally configured for range, filtering and output
* data rate before sampling values.
*/
#ifndef ISM330_H_
#define ISM330_H_
#include "mbed.h"
#include <tuple>

/**
 * @class ISM330
 * @brief Interface class for ISM330DHCX IMU using SPI.
 */
class ISM330 {
public:

    ISM330(I2C &i2c, uint8_t address) noexcept;

    /**
    * @brief Initialize communication with the device.
    *
    * Performs basic SPI setup and verifies device presence using the WHO_AM_I
    * register.
    *
    * @return true if the device identifier matches the expected chip ID.
    * @return false if device detection fails.
    */
    bool begin() noexcept;

    /**
    * @brief Read the WHO_AM_I register value.
    *
    * This value should match the documented device ID. Mainly useful for
    * diagnostics or verifying hardware connectivity.
    *
    * @return 8-bit device identifier.
    */
    uint8_t whoAmI() noexcept;

    /**
    * @brief Issue a software reset to the device.
    *
    * Triggers a soft reset and blocks briefly to allow the device to restart.
    */
    void reset() noexcept;

    //Accel and Gyro Reading functions

    // Structs
    
    // Acceleration Vector (just be careful when you use it, could be used for linear and angular)

    typedef struct {
        float x;
        float y;
        float z;
    } ISM330_VECTOR_TypeDef;

    // Euler Angles
    typedef struct{
        float yaw;
        float roll;
        float pitch;
    } ISM330_ANGULAR_POSITION_typedef;


    //Full read functions

    //Reads Accel and Gyro sequentially, reduces I2C transactions
    std::tuple<float, float, float, float, float, float> readAG() noexcept; 

    // Update Structs
    void getAGVectors(ISM330_VECTOR_TypeDef& accel, ISM330_VECTOR_TypeDef& gyro);

    // IMU Sensor Fusion
    std::tuple<float, float> imuKalmanUpdate(float accelX, float accelY, float accelZ, float gyroX, float gyroY);

    void getEulerAngles(ISM330_ANGULAR_POSITION_typedef& angles);


    private:
    I2C &i2c;
    uint8_t _address;
    uint8_t whoAmIReading;

    
    //Raw I2C readings to actual measurements
    std::tuple<float, float, float> readingToAccel(const uint8_t *readings);
    std::tuple<float, float, float> readingToGyro(const uint8_t *readings);

    //Reading Accel one at a time
    std::tuple<float, float, float> readAccel() noexcept;
    std::tuple<float, float, float> readGyro() noexcept;
};


#endif // ISM330_H_