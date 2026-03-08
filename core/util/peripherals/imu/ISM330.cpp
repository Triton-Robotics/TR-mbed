#include "mbed.h"
#include "ISM330.h"
#include <cmath>
#include <cstdint>
#include <tuple>

static constexpr uint8_t ISM330_CHIP_ID = 0x6B;   // Expected WHO_AM_I
const char WhoAmIReg[] = {0x0F};

//Reading and Write Addresses
const int readAddr = 0b11010101;
const int writeAddr = 0b11010100;

const char SWRST[] = {0x12,0x01}; // Software Reset command
const char xEnable[] = {0x10, 0x7A}; // Enable accelerometer at 833Hz, +/-4g
const char xEnableMax[] = {0x10, 0xAA}; // Enable accelerometer at 
const char gEnable[] = {0x11, 0x7C}; // Enable gyroscope at 833Hz, 2000dps
const char gEnableMax[] = {0x11, 0xAC}; // Enable gyroscope at 833Hz, 2000dps, with low pass filter

// Accelerometer and gyro register definitions
static constexpr uint8_t CTRL1_XL = 0x10;
static constexpr uint8_t CTRL2_G  = 0x11;
static constexpr uint8_t CTRL3_C  = 0x12;
static constexpr uint8_t CTRL4_C  = 0x13;
static constexpr uint8_t CTRL6_C  = 0x15;
static constexpr uint8_t STATUS_REG = 0x1E;
const char gReg[] = {0x22,0x23,0x24,0x25,0x26,0x27}; //Gyro Register values
const char xReg[] = {0x28,0x29,0x2A, 0x2B, 0x2C, 0x2D,}; //Accel Register Values
const char gyroLowPassFilter[] = {CTRL6_C, 0x03}; // Low pass filter
const char blockUpdate[] = {CTRL3_C, 0x44}; // Block update for reading consistent data

// ------------------- CONSTRUCTOR -------------------
ISM330::ISM330(I2C &i2c, uint8_t address)  //Put in an i2c object, and the device address (in this case 0x6B)
        : i2c(i2c), _address(address)
    {
    }

// ------------------- INITIALIZATION -------------------


bool ISM330::begin() noexcept
{

    i2c.frequency(400000);

    i2c.write(writeAddr, SWRST, 2, false); // Must be false on write, true on reads according to Datasheet
    ThisThread::sleep_for(100ms);

    i2c.write(writeAddr, xEnable, 2, false);

    i2c.write(writeAddr, gEnable, 2, false);

    i2c.write(writeAddr, gyroLowPassFilter, 2, false);

    i2c.write(writeAddr, blockUpdate, 2, false);
    
    i2c.write(writeAddr, WhoAmIReg, 1, false);
    
    i2c.read(readAddr, reinterpret_cast<char*>(&whoAmIReading), 1, true);

    printf("WHO_AM_I: 0x%02X\r\n", whoAmIReading);

    madgwickStart(1.0f);
    mahonyStart(0.3f,0.0f);

    calibrate();
    return true;
}

void ISM330::calibrate() noexcept{
    // Simple calibration routine, just takes 100 readings and averages them to find bias values for accel and gyro
    float axBias = 0;
    float ayBias = 0;
    float azBias = 0;
    float wxBias = 0;
    float wyBias = 0;
    float wzBias = 0;

    for (int i = 0; i < 100; i++) {
        auto [ax, ay, az, wx, wy, wz] = readAGraw();
        axBias += ax;
        ayBias += ay;
        azBias += az;
        wxBias += wx;
        wyBias += wy;
        wzBias += wz;
        ThisThread::sleep_for(10ms);
    }

    axBias /= 100.0f;
    ayBias /= 100.0f;
    azBias /= 100.0f;
    wxBias /= 100.0f;
    wyBias /= 100.0f;
    wzBias /= 100.0f;
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

// static constexpr float axBias = -0.22277; // Calculated bias values for xyz accel and gyro using Ozone
// static constexpr float ayBias = -0.30137;
// static constexpr float azBias = 9.8804 - 9.81;

// static constexpr float wxBias = 0.0054391;
// static constexpr float wyBias = -0.0097426;
// static constexpr float wzBias = -0.0061490;



//Raw values to Acceleration (m/s^2)
std::tuple<float, float, float> ISM330::readingToAccel(const uint8_t *readings) {
    int16_t rawX = (int16_t)(((uint16_t)readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)(((uint16_t)readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)(((uint16_t)readings[5] << 8) | readings[4]);

    float rawXfloat = static_cast<float>(rawX);
    float rawYfloat = static_cast<float>(rawY);
    float rawZfloat = static_cast<float>(rawZ);

    float x = ((rawXfloat-axBias) * accelConversion);
    float y = ((rawYfloat-ayBias) * accelConversion);
    float z = ((rawZfloat-azBias) * accelConversion);


    return std::make_tuple(x, y, z);
}

//Raw to Gyro (Radians/second)

std::tuple<float, float, float> ISM330::readingToGyro(const uint8_t *readings) {
    int16_t rawX = (int16_t)(((uint16_t)readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)(((uint16_t)readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)(((uint16_t)readings[5] << 8) | readings[4]);

    float rawXfloat = static_cast<float>(rawX);
    float rawYfloat = static_cast<float>(rawY);
    float rawZfloat = static_cast<float>(rawZ);


    float x = ((rawXfloat-wxBias) * gyroConversion); 
    float y = ((rawYfloat-wyBias) * gyroConversion);
    float z = ((rawZfloat-wzBias) * gyroConversion);

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

std::tuple<float, float, float, float, float, float> ISM330::readAGraw() noexcept {
    i2c.write(writeAddr, gReg, 1, false); 
    i2c.read(readAddr, reinterpret_cast<char*>(agReadings), 12, true);

    int16_t rawGX = (int16_t)((agReadings[1] << 8) | agReadings[0]);
    int16_t rawGY = (int16_t)((agReadings[3] << 8) | agReadings[2]);
    int16_t rawGZ = (int16_t)((agReadings[5] << 8) | agReadings[4]);

    int16_t rawAX = (int16_t)((agReadings[7] << 8) | agReadings[6]);
    int16_t rawAY = (int16_t)((agReadings[9] << 8) | agReadings[8]);
    int16_t rawAZ = (int16_t)((agReadings[11] << 8) | agReadings[10]);

    return std::make_tuple(rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ);
}



std::tuple<float, float, float, float, float, float> ISM330::readAG() noexcept//
{
    i2c.write(writeAddr, gReg, 1, false); 
    i2c.read(readAddr, reinterpret_cast<char*>(agReadings), 12, true);

    // printf("Raw Readings: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,\r\n", 
    // agReadings[0], agReadings[1], agReadings[2], agReadings[3], agReadings[4], agReadings[5],
    // agReadings[6], agReadings[7], agReadings[8], agReadings[9], agReadings[10], agReadings[11]
    // );

    auto [gx,gy,gz] = readingToGyro(&agReadings[0]);
    auto [ax,ay,az] = readingToAccel(&agReadings[6]);

    ISM330_VECTOR_TypeDef accelVector = {ax, ay, az};
    ISM330_VECTOR_TypeDef gyroVector = {gx, gy, gz};

    return std::make_tuple(ax, ay, az, gx, gy, gz);
}
    void ISM330::getXLVector(ISM330_VECTOR_TypeDef& accelVector) {
        auto [ax, ay, az] = readAccel();
        accelVector.x = ax;
        accelVector.y = ay;
        accelVector.z = az;
    }

    void ISM330::getGyroVector(ISM330_VECTOR_TypeDef& gyroVector) {
        auto [gx, gy, gz] = readGyro();
        gyroVector.x = gx;
        gyroVector.y = gy;
        gyroVector.z = gz;
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


// Madgwick Stuff (https://github.com/adafruit/Adafruit_AHRS/blob/master/src/Adafruit_AHRS_Madgwick.cpp)


// The original Madgwick algorithm used a fast inverse square root approx, but for accuracy we can just do the real thing I think
// If we need speed later, we can always switch to the approx
float ISM330::invSqrt(float x) { 
    return 1/sqrt(x);

    //For easy access later, here's the approx

//     float halfx = 0.5f * x;
//   union {
//     float f;
//     long i;
//   } conv = {x};
//   conv.i = 0x5f3759df - (conv.i >> 1);
//   conv.f *= 1.5f - (halfx * conv.f * conv.f);
//   conv.f *= 1.5f - (halfx * conv.f * conv.f);
//   return conv.f;
}

void ISM330::madgwickStart(float gain) {
  beta = gain;
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
//   invSampleFreq = 1.0f / sampleFreqDef;
  anglesComputed = false;
}


void ISM330::madgwickUpdate(float mx, float my, float mz, float dt) {
    auto [ax, ay, az, gx, gy, gz] = readAG();

    //Converting to g/s instead of m/s^2
    ax /= g;
    ay /= g;
    az /= g;

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
      _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
      q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
  // magnetometer normalisation)
      if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        madgwickUpdateIMU(dt);
        return;
      }



  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 +
         _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 +
         my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
           _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3 + _2bz * q1) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2 + _2bz * q0) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3 - _4bz * q1) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2 - _2bz * q0) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q1 + _2bz * q3) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0 - _4bz * q2) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * q3 + _2bz * q1) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q0 + _2bz * q2) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  anglesComputed = 0;
}

void ISM330::madgwickUpdateIMU (float dt) {
    auto [ax, ay, az, gx, gy, gz] = readAG();
    //Converting to g/s instead of m/s^2

    // float ax_sum = 0;
    // float ay_sum = 0;
    // float az_sum = 0;
    // float gx_sum = 0;
    // float gy_sum = 0;
    // float gz_sum = 0;

    // for (int i = 1; i < 10; i++) {
    //     auto [ax, ay, az, gx, gy, gz] = readAG();
    //     ax_sum = ax;
    //     ay_sum = ay;
    //     az_sum = az;
        
    //     gx_sum = gx;
    //     gy_sum = gy;
    //     gz_sum = gz;
    // }
    // float ax = ax_sum;
    // float ay = ay_sum;
    // float az = az_sum;

    // float gx = gx_sum;
    // float gy = gy_sum;
    // float gz = gz_sum;

    ax /= g;
    ay /= g;
    az /= g;

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;



  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  anglesComputed = 0;
} 


void ISM330::computeAngles(ISM330_ANGULAR_POSITION_typedef& angles) {
  angles.roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2)*radToDeg;
  angles.pitch = asinf(-2.0f * (q1 * q3 - q0 * q2))*radToDeg;
  angles.yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3)*radToDeg;
  grav[0] = 2.0f * (q1 * q3 - q0 * q2);
  grav[1] = 2.0f * (q0 * q1 + q2 * q3);
  grav[2] = 2.0f * (q0 * q0 - 0.5f + q3 * q3);
  anglesComputed = 1;
}


// Mahony Sensor fusion

void ISM330::mahonyStart(float prop_gain, float int_gain) {
  twoKp = prop_gain; // 2 * proportional gain (Kp)
  twoKi = int_gain;  // 2 * integral gain (Ki)
  mahq0 = 1.0f;
  mahq1 = 0.0f;
  mahq2 = 0.0f;
  mahq3 = 0.0f;
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
  anglesComputed = false;
  //invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;
}


void ISM330::mahonyUpdate(float mx, float my, float mz, float dt) {
    auto [ax, ay, az, gx, gy, gz] = readAG();

    //Converting to g/s instead of m/s^2
    ax /= g;
    ay /= g;
    az /= g;
  
    float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Use IMU algorithm if magnetometer measurement invalid
  // (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    mahonyUpdateIMU(dt);
    return;
  }

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = mahq0 * mahq0;
    q0q1 = mahq0 * mahq1;
    q0q2 = mahq0 * mahq2;
    q0q3 = mahq0 * mahq3;
    q1q1 = mahq1 * mahq1;
    q1q2 = mahq1 * mahq2;
    q1q3 = mahq1 * mahq3;
    q2q2 = mahq2 * mahq2;
    q2q3 = mahq2 * mahq3;
    q3q3 = mahq3 * mahq3;

    // Reference direction of Earth's magnetic field
    hx = 2.0f *
         (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f *
         (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f *
         (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction
    // and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      // integral error scaled by Ki
      integralFBx += twoKi * halfex * dt;
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt); // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = mahq0;
  qb = mahq1;
  qc = mahq2;
  mahq0 += (-qb * gx - qc * gy - mahq3 * gz);
  mahq1 += (qa * gx + qc * gz - mahq3 * gy);
  mahq2 += (qa * gy - qb * gz + mahq3 * gx);
  mahq3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(mahq0 * mahq0 + mahq1 * mahq1 + mahq2 * mahq2 + mahq3 * mahq3);
  mahq0 *= recipNorm;
  mahq1 *= recipNorm;
  mahq2 *= recipNorm;
  mahq3 *= recipNorm;
  anglesComputed = 0;
}

void ISM330::mahonyUpdateIMU(float dt) {
    auto [ax, ay, az, gx, gy, gz] = readAG();
    
    //Converting to g/s instead of m/s^2
    ax /= g;
    ay /= g;
    az /= g;


  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;



  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = mahq1 * mahq3 - mahq0 * mahq2;
    halfvy = mahq0 * mahq1 + mahq2 * mahq3;
    halfvz = mahq0 * mahq0 - 0.5f + mahq3 * mahq3;

    // Error is sum of cross product between estimated
    // and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      // integral error scaled by Ki
      integralFBx += twoKi * halfex * dt;
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx; // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt); // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = mahq0;
  qb = mahq1;
  qc = mahq2;
  mahq0 += (-qb * gx - qc * gy - mahq3 * gz);
  mahq1 += (qa * gx + qc * gz - mahq3 * gy);
  mahq2 += (qa * gy - qb * gz + mahq3 * gx);
  mahq3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(mahq0 * mahq0 + mahq1 * mahq1 + mahq2 * mahq2 + mahq3 * mahq3);
  mahq0 *= recipNorm;
  mahq1 *= recipNorm;
  mahq2 *= recipNorm;
  mahq3 *= recipNorm;
  anglesComputed = 0;
}

void ISM330::computeAnglesMah(ISM330_ANGULAR_POSITION_typedef& angles) {
  angles.roll = atan2f(mahq0 * mahq1 + mahq2 * mahq3, 0.5f - mahq1 * mahq1 - mahq2 * mahq2)*radToDeg;
  angles.pitch = asinf(-2.0f * (mahq1 * mahq3 - mahq0 * mahq2))*radToDeg;
  angles.yaw = atan2f(mahq1 * mahq2 + mahq0 * mahq3, 0.5f - mahq2 * mahq2 - mahq3 * mahq3)*radToDeg;
  grav[0] = 2.0f * (mahq1 * mahq3 - mahq0 * mahq2);
  grav[1] = 2.0f * (mahq0 * mahq1 + mahq2 * mahq3);
  grav[2] = 2.0f * (mahq0 * mahq0 - 0.5f + mahq3 * mahq3);
  anglesComputed = 1;
}

