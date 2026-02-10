#include "mbed.h"
#include "ISM330.h"
#include <cstdint>

static constexpr uint8_t ISM330_CHIP_ID = 0x6B;   // Expected WHO_AM_I
const char WhoAmIReg[] = {0x0F};

//Reading and Write Addresses
const int readAddr = 0b11010101;
const int writeAddr = 0b11010100;

const char SWRST[] = {0x12,0x01}; // Software Reset command
const char xEnable[] = {0x10, 0x7A}; // Enable accelerometer at 833Hz, +/-4g
const char gEnable[] = {0x11, 0x7C}; // Enable gyroscope at 833Hz, 2000dps

// Accelerometer and gyro register definitions
static constexpr uint8_t CTRL1_XL = 0x10;
static constexpr uint8_t CTRL2_G  = 0x11;
static constexpr uint8_t CTRL3_C  = 0x12;
static constexpr uint8_t CTRL4_C  = 0x13;
static constexpr uint8_t STATUS_REG = 0x1E;
const char gReg[] = {0x22,0x23,0x24,0x25,0x26,0x27}; //Gyro Register values
const char xReg[] = {0x28,0x29,0x30,0x31,0x32,0x33}; //Accel Register Values


// ------------------- CONSTRUCTOR -------------------
ISM330::ISM330(I2C &i2c, uint8_t address)  //Put in an i2c object, and the device address (in this case 0x6B)
        : i2c(i2c), _address(address)
    {
    }

// ------------------- INITIALIZATION -------------------


bool ISM330::begin() noexcept
{

    i2c.frequency(100000);

    i2c.write(writeAddr, SWRST, 2, false); // Must be false on write, true on reads according to Datasheet
    ThisThread::sleep_for(100ms);

    i2c.write(writeAddr, xEnable, 2, false);

    i2c.write(writeAddr, gEnable, 2, false);
    
    i2c.write(writeAddr, WhoAmIReg, 1, false);
    
    i2c.read(readAddr, reinterpret_cast<char*>(&whoAmIReading), 1, true);

    printf("WHO_AM_I: 0x%02X\r\n", whoAmIReading);
    return true;
}


// ------------------- BASIC DEBUG FUNCTIONS -------------------
uint8_t ISM330::whoAmI() noexcept
{
    i2c.write(writeAddr, WhoAmIReg, 1, false);
    char whoAmIReading;
    i2c.read(readAddr, reinterpret_cast<char*>(&whoAmIReading), 1, true);
    return whoAmIReading;

}

void ISM330::reset() noexcept
{
    i2c.write(writeAddr, SWRST, 2, false); 
    ThisThread::sleep_for(100ms);
}

//i2c.write and read are simple enough that we don't need private functions for them
//unlike SPI which is more annoying


// ------------------- SENSOR READOUT -------------------

//Helper functions to convert accel readings to real values

// Scale factors for calculations 
static constexpr float accelRangeScale = 0.122f;  // mg/LSB @ ±4g; see datasheet for other ranges
static constexpr float gyroRangeScale  = 70.0f;   // mdps/LSB @ ±2000 dps

static constexpr float g = 9.80665f; 
static constexpr float dpsToRad = 3.14159265358979323846f / 180.0f;
static constexpr float radToDeg = 180.0f / 3.14159265358979323846f;

static constexpr float accelConversion = accelRangeScale * g / 1000.0f; // The actual conversion factor from raw to m/s^2
static constexpr float gyroConversion = gyroRangeScale * dpsToRad / 1000.0f;   // The conversion factor from raw to rad/s

//Raw values to Acceleration (m/s^2)
std::tuple<float, float, float> ISM330::readingToAccel(const uint8_t *readings) {
    int16_t rawX = (int16_t)((readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)((readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)((readings[5] << 8) | readings[4]);


    float x = rawX * accelConversion;
    float y = rawY * accelConversion;
    float z = rawZ * accelConversion;


    return std::make_tuple(x, y, z);
}

//Raw to Gyro (Radians/second)

std::tuple<float, float, float> ISM330::readingToGyro(const uint8_t *readings) {
    int16_t rawX = (int16_t)((readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)((readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)((readings[5] << 8) | readings[4]);

    float x = rawX * gyroConversion;
    float y = rawY * gyroConversion;
    float z = rawZ * gyroConversion;

    return std::make_tuple(x, y, z);
}

// Actual Reading functions
uint8_t xReadings[6]; //Accel Readings buffer
std::tuple<float, float, float> ISM330::readAccel() noexcept
{
    i2c.write(writeAddr, xReg, 1, false);
    i2c.read(readAddr, reinterpret_cast<char*>(xReadings), 6, true);
    
    auto [x,y,z] = readingToAccel(xReadings);
    return std::make_tuple(x, y, z);
}

uint8_t gReadings[6]; //Gyro Readings buffer
std::tuple<float, float, float> ISM330::readGyro() noexcept
{
    i2c.write(writeAddr, gReg, 1, false);
    i2c.read(readAddr, reinterpret_cast<char*>(gReadings), 6, true);

    auto [x,y,z] = readingToGyro(gReadings);
    return std::make_tuple(x, y, z);
}


// Combined Read function, should be more efficient because less I2C transactions
uint8_t agReadings[12]; //Accel and Gyro Readings buffer;
std::tuple<float, float, float, float, float, float> ISM330::readAG() noexcept//
{
    i2c.write(writeAddr, gReg, 1, false); 
    i2c.read(readAddr, reinterpret_cast<char*>(agReadings), 12, true);

    auto [gx,gy,gz] = readingToGyro(&agReadings[0]);
    auto [ax,ay,az] = readingToAccel(&agReadings[6]);

    ISM330_VECTOR_TypeDef accelVector = {ax, ay, az};
    ISM330_VECTOR_TypeDef gyroVector = {gx, gy, gz};

    return std::make_tuple(ax, ay, az, gx, gy, gz);
}

    void ISM330::getAGVectors(ISM330_VECTOR_TypeDef& accel, ISM330_VECTOR_TypeDef& gyro){
    auto [ax, ay, az, gx, gy, gz] = readAG();
    accel.x = ax;
    accel.y = ay;
    accel.z = az;
    gyro.x = gx;
    gyro.y = gy;
    gyro.z = gz;
    }




// Sensor Fusion Stuff; Stolen from https://sparxeng.com/blog/software/imu-signal-processing-with-kalman-filter


// Variable Definitions


    // Angular Velocities in x y z
    float w_x;
    float w_y;
    float w_z;

   
    // States predicted by model (regarding the Gyroscope measurements as inputs)
    float theta_m = 0;
    float phi_m = 0;
   
    // States measured by sensor (accelerometer)
    float theta_s;  // Pitch
    float phi_s;    // roll
    float psi_s = 0;    // Yaw, we don't use kalman, we just integrate w_z
   
    // Kalman Filter parameters and initailization
    float theta_n; // a priori estimation of Theta
    float theta_p = 0; // a posteriori estimation on Theta (set to zero for the initial time step k=0)
    float phi_n; // a priori estimation of Phi
    float phi_p = 0; // a posteriori estimation on Phi (set to zero for the initial time step k=0)

   
    float P_theta_n; // a priori covariance of Theta
    float P_phi_n; // a priori covariance of Phi
    float P_theta_p = 0; // a posteriori covariance of Theta
    float P_phi_p = 0; // a posteriori covariance of Phi
   
    float K_theta; // Observer gain or Kalman gain for Theta
    float K_phi; // Observer gain or Kalman gain for Phi
   
    float Q = 0.1; // Covariance of disturbance (unknown disturbance affecting w_x and w_y)
    float R = 4; // Covariance of noise (unknown noise affecting theta_s and phi_s)


    float sampleRate = 1/833.0;   //0.001 // Assuming 1khz for now
    float ds = sampleRate *0.001; // Time step, not exactly sure why its 1/1000 sample rate but hopefully its good?

    //Timer stuff for integration
    static Timer timer;
    static bool timerStarted = false;
    
    
std::tuple<float, float, float> ISM330::imuKalmanUpdate(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
    // Updating inputs
    w_x = gyroX; // input
    w_y = gyroY; // input
    w_z = gyroZ; // input

    //Timer for yaw 
    if (!timerStarted) {
        timer.start();
        timerStarted = true;
    }

    // Updating models and the sensor measurements
    theta_m = theta_m - w_y*0.1;
    phi_m = phi_m + w_x*0.1;


    theta_s=atan2(accelX/9.8,accelZ/9.8)/2/3.141592654*360;
    phi_s=atan2(accelY/9.8,accelZ/9.8)/2/3.141592654*360;  
   
    // Predicting Pitch (theta), Roll (phi), and Yaw (psi) using Kalman Filter equations
    P_theta_n = P_theta_p + Q;
    K_theta = P_theta_n/(P_theta_n + R);
    theta_n = theta_p - ds*w_y;
    theta_p = theta_n + K_theta*(theta_s - theta_n);
    P_theta_p = (1-K_theta)*P_theta_n;
   
    P_phi_n = P_phi_p + Q;
    K_phi = P_phi_n/(P_phi_n + R);
    phi_n = phi_p + ds*w_x;
    phi_p = phi_n + K_phi*(phi_s - phi_n);
    P_phi_p = (1-K_phi)*P_phi_n;

    float dt = timer.elapsed_time().count() * 0.000001;
    timer.reset();
    psi_s = psi_s + w_z*dt; // Just integrate the gyro for yaw, no Kalman filter for that
    
    return std::make_tuple(theta_p, phi_p, psi_s);
}


void ISM330::getEulerAngles(ISM330_ANGULAR_POSITION_typedef& angles) {
    auto [ax, ay, az, gx, gy, gz] = readAG();


    auto [pitch, roll, yaw] = imuKalmanUpdate(ax, ay, az, gx, gy, gz);
    angles.pitch = pitch;
    angles.roll = roll;
    angles.yaw = yaw * radToDeg;
}
