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

    // Accel and Gyroscale variables (class-scope so they can be referenced
    // as `ISM330::accelScale` from other compilation units if needed)

    /**
    * @brief Construct a new ISM330 object.
    *
    * Chip select is set high after initialization to ensure the device is
    * deselected.
    *
    * @param spi Reference to an initialized SPI peripheral.
    * @param csPin GPIO pin used as the chip-select line (redundant).
    */
    ISM330(SPI &spi, PinName csPin) noexcept;

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

    /**
    * @brief Read the latest accelerometer sample.
    *
    * Data returned in X, Y, Z order.
    *
    * @return A tuple of acceleration values in units defined by the active
    *         sensor full-scale range.
    */
    [[nodiscard]] std::tuple<float, float, float> getAccel() noexcept;

    /**
    * @brief Read the latest gyroscope sample.
    *
    * Data returned in X, Y, Z order.
    *
    * @return A tuple of angular velocity values in units defined by the active
    *         sensor full-scale range.
    */
    [[nodiscard]] std::tuple<float, float, float> getGyro() noexcept;

    /**
    * @deprecated Use getAccel() instead.
    *
    * @brief Read accelerometer values using pointer parameters.
    */
    [[deprecated("use `getAccel()` instead.")]]
    void readAccel(float *x, float *y, float *z) noexcept;

    /**
    * @deprecated Use getGyro() instead.
    *
    * @brief Read gyroscope values using pointer parameters.
    */
    [[deprecated("use `getGyro()` instead.")]]
    void readGyro(float *x, float *y, float *z) noexcept;

    void setAccelRange(int r) noexcept; 
    // Setting the range from +/- 2G to 16G

    void setGyroRange(int r) noexcept;
    // Setting Gyrorange from 250 to 2000DPS 

    private:
    
    //char txBuffer[8]; // write buffer for calibration
    //char rxBuffer[30]; // read buffer
    /**
     * @brief Read a single register from the device.
     *
     * @param reg Register address to read.
    * @return The value stored in the register.
    */
   uint8_t readRegister(uint8_t reg) noexcept;

   /**
    * @brief Read multiple consecutive registers starting from a base address.
    *
    * CS handling is performed internally. The caller must provide a buffer
    * large enough to hold `len` bytes.
    *
    * @param reg Starting register address.
    * @param buf Destination buffer.
    * @param len Number of bytes to read.
    */
   void readMultiple(uint8_t reg, uint8_t *buf, uint8_t len) noexcept;
   
   /**
    * @brief Write a single register value.
    *
    * @param reg Register to modify.
    * @param value Value to write.
    */
   void writeRegister(uint8_t reg, uint8_t value) noexcept;

    SPI &_spi; ///< Reference to SPI bus instance.
    DigitalOut _cs; ///< Chip-select GPIO line (active low).

    // Register Maps for readability

    enum Register : uint8_t
{
    STATUS_REG = 0x1E, //Indicates if data is ready (first bit is for XL, 2nd is for gyroscope, 3rd is for temp)
    /* temperature; Currently not in use yet, but in the future we may want to make a 
    polynomial/look up table accounting for temp differences
    */
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,

    // gyroscope
    CTRL2_G = 0x11,
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,

    // accelerometer
    CTRL1_XL = 0x10,
    OUTX_L_XL = 0x28,
    OUTX_H_XL = 0x29,
    OUTY_L_XL = 0x2A,
    OUTY_H_XL = 0x2B,
    OUTZ_L_XL = 0x2C,
    OUTZ_H_XL = 0x2D
};


};


#endif // ISM330_H_