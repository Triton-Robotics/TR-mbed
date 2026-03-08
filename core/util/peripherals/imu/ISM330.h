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
#include "util/peripherals/imu/BNO055.h"
#include <cstdint>
#include <tuple>


/**
 * @class ISM330
 * @brief Interface class for ISM330DHCX IMU using SPI.
 */
class ISM330 : public IMU {
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

    void calibrate() noexcept; 

    // Raw offsets for accel and gyro
    float axBias;
    float ayBias;
    float azBias;
    float wxBias;
    float wyBias;
    float wzBias;



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
    std::tuple<float, float, float, float, float, float> readAGraw() noexcept; //used during calibration

    std::tuple<float, float, float, float, float, float> readAG() noexcept;


    // Update Structs
    void getAGVectors(ISM330_VECTOR_TypeDef& accel, ISM330_VECTOR_TypeDef& gyro);


    // IMU Sensor Fusion
    std::tuple<float, float, float> imuKalmanUpdate(float dt, float psi_s);


    void getEulerAngles(ISM330_ANGULAR_POSITION_typedef& angles, float dt, float psi_s);

    void getXLVector(ISM330_VECTOR_TypeDef& accelVector);

    void getGyroVector(ISM330_VECTOR_TypeDef& gyroVector);

    // Madgwick Sensor Fusion

    void madgwickStart(float gain);
    
    void madgwickUpdate(float mx, float my, float mz, float dt);
    
    
    void madgwickUpdateIMU(float dt);
    
    
    void computeAngles(ISM330_ANGULAR_POSITION_typedef& angles); //Madgwick Compute Euler Anglse

    IMU::EulerAngles getImuAngles() override;

    IMU::EulerAngles read() override { return getImuAngles(); }

    void computeAnglesMah(ISM330_ANGULAR_POSITION_typedef& angles); //Mahony Compute Euler Angles
    
    // Mahony Sensor Fusion
    //Adafruit_Mahony(float prop_gain, float int_gain);
    void mahonyStart(float prop_gain, float int_gain);
    
    void mahonyUpdate(float mx, float my, float mz, float dt);
    
    void mahonyUpdateIMU(float dt);
    
    float getKp() { return twoKp / 2.0f; }
    
    void setKp(float Kp) { twoKp = 2.0f * Kp; }
    
    float getKi() { return twoKi / 2.0f; }
    
    void setKi(float Ki) { twoKi = 2.0f * Ki; }
    
    
    
    
    
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
  float integralFBx, integralFBy,
      integralFBz; // integral error terms scaled by Ki

bool anglesComputed = false; // Flag for whether or not angles have been computed
};




#endif // ISM330_H_