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
static constexpr float dpsToRad = M_PI / 180.0f;
static constexpr float radToDeg = 180.0f / M_PI;

static constexpr float accelConversion = accelRangeScale * g / 1000.0f; // The actual conversion factor from raw to m/s^2
static constexpr float gyroConversion = gyroRangeScale * dpsToRad / 1000.0f;   // The conversion factor from raw to rad/s

static constexpr float axBias = -0.22277; // Calculated bias values for xyz accel and gyro using Ozone
static constexpr float ayBias = -0.30137;
static constexpr float azBias = 9.8804 - 9.81;

static constexpr float wxBias = 0.0054391;
static constexpr float wyBias = -0.0097426;
static constexpr float wzBias = -0.0061490;



//Raw values to Acceleration (m/s^2)
std::tuple<float, float, float> ISM330::readingToAccel(const uint8_t *readings) {
    int16_t rawX = (int16_t)((readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)((readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)((readings[5] << 8) | readings[4]);


    float x = (rawX * accelConversion) -axBias;
    float y = (rawY * accelConversion) -ayBias;
    float z = (rawZ * accelConversion) -azBias;


    return std::make_tuple(x, y, z);
}

//Raw to Gyro (Radians/second)

std::tuple<float, float, float> ISM330::readingToGyro(const uint8_t *readings) {
    int16_t rawX = (int16_t)((readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)((readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)((readings[5] << 8) | readings[4]);

    float x = (rawX * gyroConversion) - wxBias; 
    float y = (rawY * gyroConversion) - wyBias;
    float z = (rawZ * gyroConversion) - wzBias;

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

   
    // States predicted by model (regarding the Gyroscope measurements as inputs)
    float theta_m = 0;
    float phi_m = 0; //This isn't actually used, but its in the code we stole
   
    // States measured by sensor (accelerometer)
    float theta_s;  // Pitch
    float phi_s;    // roll
    float psi_s;    // Yaw
   
    // Kalman Filter parameters and initailization
    float theta_n; // a priori estimation of Theta
    float theta_p = 0; // a posteriori estimation on Theta (set to zero for the initial time step k=0)
    float phi_n; // a priori estimation of Phi
    float phi_p = 0; // a posteriori estimation on Phi (set to zero for the initial time step k=0)
    float psi_n; // a priori estimation of Psi, not really used since we aren't doing Kalman for yaw
    float psi_p = 0; // a posteriori estimation of Psi, not really used

    float P_theta_n; // a priori covariance of Theta
    float P_phi_n; // a priori covariance of Phi
    float P_theta_p = 0; // a posteriori covariance of Theta
    float P_phi_p = 0; // a posteriori covariance of Phi
    float P_psi_n; // a priori covariance of Psi, 
    float P_psi_p = 0; // a posteriori covariance of Psi, 
   
    float K_theta; // Observer gain or Kalman gain for Theta
    float K_phi; // Observer gain or Kalman gain for Phi
    float K_psi; // Observer gain or Kalman gain for Psi
   
    float Q = 0.1; // ORIGINALLY 0.1 Covariance of disturbance (unknown disturbance affecting w_x and w_y)
    float R = 4; // Covariance of noise (unknown noise affecting theta_s and phi_s)

    float R_yaw = 4;

    float axVariance = 0.00040560; // We tried looking at variance of each in isolation, but it wasn't effective. Possibly do covariance in the future
    float ayVariance = 0.00036644;
    float azVariance = 0.0029198;

    float sampleRate = 1/833.0;   // Since ODR is 833hz
    float ds = sampleRate *0.001; // converting to ms/frequency 

    // //Timer stuff for integration
    // static Timer timer;
    // static bool timerStarted = false;
    
    
std::tuple<float, float, float> ISM330::imuKalmanUpdate(float dt, float psi_s) {
    // Updating inputs
    auto [ax, ay, az, w_x, w_y, w_z] = readAG();

    // Updating models and the sensor measurements
    theta_m = theta_m - w_y*0.1;
    phi_m = phi_m + w_x*0.1;


    theta_s=atan2(ax/9.8,az/9.8)/2/M_PI*360;
    phi_s=atan2(ay/9.8,az/9.8)/2/M_PI*360;  

    // Predicting Pitch (theta), Roll (phi), and Yaw (psi) using Kalman Filter equations
    P_theta_n = P_theta_p + Q;
    K_theta = P_theta_n/(P_theta_n + R); //Formally used +R
    theta_n = theta_p - ds*w_y;
    theta_p = theta_n + K_theta*(theta_s - theta_n); 
    P_theta_p = (1-K_theta)*P_theta_n;

    P_phi_n = P_phi_p + Q;
    K_phi = P_phi_n/(P_phi_n + R); //Formally used +R
    phi_n = phi_p + ds*w_x;
    phi_p = phi_n + K_phi*(phi_s - phi_n);
    P_phi_p = (1-K_phi)*P_phi_n;

    // --- Yaw Kalman Filter ---

    P_psi_n = P_psi_p + Q;
    K_psi = P_psi_n/(P_psi_n + R_yaw); //Formally used +R
    psi_n = psi_p + dt * w_z;  // prediction from ISM gyro
    
    // Wrap Around 
    float error = psi_s - psi_n;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    psi_p = (1 - 0.1) * (psi_p + dt*w_z) + 0.1 * psi_s;

    //psi_p = psi_n + K_psi*(psi_s - psi_n);
    P_psi_p = (1 - K_psi) * P_psi_n;


    
    // float dt = timer.elapsed_time().count() * 0.000001;
    // timer.reset();
    //psi_s = psi_s + w_z*dt; // Just integrate the gyro for yaw, no Kalman filter for that
    
    return std::make_tuple(theta_p, phi_p, psi_p);
}


void ISM330::getEulerAngles(ISM330_ANGULAR_POSITION_typedef& angles, float dt, float psi_s) {
    auto [pitch, roll, yaw] = imuKalmanUpdate(dt, psi_s);
    angles.pitch = pitch; 
    angles.roll = roll;
    angles.yaw = yaw; // Use * radToDeg / 1000 when just integrating
}
