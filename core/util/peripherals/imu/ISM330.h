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
    
    //Raw I2C readings to actual measurements
    std::tuple<int16_t, int16_t, int16_t> readingToAccel(const uint8_t *readings);
    std::tuple<int16_t, int16_t, int16_t> readingToGyro(const uint8_t *readings);

    //Full read functions
    std::tuple<int16_t, int16_t, int16_t> readAccel() noexcept;

    std::tuple<int16_t, int16_t, int16_t> readGyro() noexcept;

    // IMU Sensor Fusion
    void imuKalmanUpdate(float accelX, float accelY, float accelZ, float gyroX, float gyroY);

    private:
    I2C &i2c;
    uint8_t _address;
    uint8_t whoAmIReading;


};


#endif // ISM330_H_