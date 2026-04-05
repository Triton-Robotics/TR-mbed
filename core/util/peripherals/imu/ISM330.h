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
#pragma once

#ifndef ISM330_H_
#define ISM330_H_
#include "mbed.h"
#include "util/peripherals/imu/IMU.h"
#include <cstdint>

/**
 * @class ISM330
 * @brief Interface class for ISM330DHCX IMU using SPI.
 */
class ISM330 : public IMU {
public:
    // Structs
    typedef struct {
        float x;
        float y;
        float z;
    } ISM330_VECTOR_TypeDef;

    typedef struct {
        int16_t ax;
        int16_t ay;
        int16_t az;
        int16_t gx;
        int16_t gy;
        int16_t gz;
    } ISM330_RAW_DATA_TypeDef;

    typedef struct {
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
    } ISM330_DATA_TypeDef;


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
    bool begin(float prop_gain=0.1f, float int_gain=0.0f) noexcept;

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
    
    void calibrate() noexcept; 
    
    IMU::EulerAngles getImuAngles() override;
    
    IMU::EulerAngles read() override { return getImuAngles(); }

   //Full read functions
    //Reads Accel and Gyro sequentially, reduces I2C transactions
    ISM330_RAW_DATA_TypeDef readAGraw() noexcept; //used during calibration

    ISM330_DATA_TypeDef readAG() noexcept;


    // Update Structs
    void getAGVectors(ISM330_VECTOR_TypeDef& accel, ISM330_VECTOR_TypeDef& gyro);

    // IMU Sensor Fusion
    void computeAnglesMah(IMU::EulerAngles& angles); // Mahony Compute Euler Angles

    void computeAnglesMadgwick(IMU::EulerAngles& angles);
    
    // Mahony Sensor Fusion
    //Adafruit_Mahony(float prop_gain, float int_gain);
    void mahonyStart(float prop_gain, float int_gain);

    void initQuaternionFromAccel();
    
    void mahonyUpdate(float mx, float my, float mz, float dt);
    
    void mahonyUpdateIMU(float dt);

    // Madgwick Sensor Fusion

    void madgwickStart(float gain);
    
    void madgwickUpdate(float mx, float my, float mz, float dt);
    
    void madgwickUpdateIMU(float dt);

    // Reading individual sensor values
    ISM330_VECTOR_TypeDef readAccel() noexcept;

    ISM330_VECTOR_TypeDef readGyro() noexcept;
    
private:
    I2C &i2c;
    uint8_t _address;
    uint8_t whoAmIReading;

    // Raw offsets for accel and gyro
    float axBias;
    float ayBias;
    float azBias;
    float wxBias;
    float wyBias;
    float wzBias;
    
    //Raw I2C readings to actual measurements
    uint8_t agReadings[12]; //Accel and Gyro Readings buffer
    uint8_t xReadings[6]; //Accel Readings buffer
    uint8_t gReadings[6]; //Gyro Readings buffer

    ISM330_VECTOR_TypeDef readingToAccel(const uint8_t *readings);
    
    ISM330_VECTOR_TypeDef readingToGyro(const uint8_t *readings);
        
    float invSqrt(float x); //Inverse square root, used for normalizing vectors
    
    float beta; // algorithm gain
    float q0;
    float q1;
    float q2;
    float q3; // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float grav[3]; // gravity vector 

    //Mahony Quaternions
    float mahq0;
    float mahq1;
    float mahq2;
    float mahq3;

    float twoKp; // 2 * proportional gain (Kp)
    float twoKi; // 2 * integral gain (Ki)
    float integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki

    bool anglesComputed = false; // Flag for whether or not angles have been computed
};

#endif // ISM330_H_