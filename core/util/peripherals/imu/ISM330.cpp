#include "mbed.h"
#include "ISM330.h"
#include <cmath>
#include <cstdint>

// Scale factors for calculations 
static constexpr float accelRangeScale = 0.122f;  // mg/LSB @ ±4g; see datasheet for other ranges
static constexpr float gyroRangeScale  = 70.0f;   // mdps/LSB @ ±2000 dps

static constexpr float g = 9.80665f; 
static constexpr float dpsToRad = M_PI / 180.0f;
static constexpr float radToDeg = 180.0f / M_PI;

static constexpr float accelConversion = accelRangeScale * g / 1000.0f; // The actual conversion factor from raw to m/s^2
static constexpr float gyroConversion = gyroRangeScale * dpsToRad / 1000.0f;   // The conversion factor from raw to rad/s


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
ISM330::ISM330(I2C &i2c, uint8_t address) noexcept  //Put in an i2c object, and the device address (in this case 0x6B)
    : i2c(i2c), _address(address) 
{
    begin();
}


// ------------------- INITIALIZATION -------------------
bool ISM330::begin(float prop_gain, float int_gain) noexcept
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

    mahonyStart(2*prop_gain,2*int_gain);
    calibrate();

    return true;
}

void ISM330::calibrate() noexcept
{
    // Simple calibration routine, just takes 1000 readings and averages them to find bias values for accel and gyro
    axBias = 0;
    ayBias = 0;
    azBias = 0;
    wxBias = 0;
    wyBias = 0;
    wzBias = 0;

    for (int i = 0; i < 1000; i++) {
        ISM330_RAW_DATA_TypeDef raw_data = readAGraw();
        axBias += raw_data.ax;
        ayBias += raw_data.ay;
        azBias += raw_data.az;
        wxBias += raw_data.gx;
        wyBias += raw_data.gy;
        wzBias += raw_data.gz;
        ThisThread::sleep_for(2ms);
    }

    axBias /= 1000.0f;
    ayBias /= 1000.0f;
    azBias /= 1000.0f;
    wxBias /= 1000.0f;
    wyBias /= 1000.0f;
    wzBias /= 1000.0f;
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

// ------------------- SENSOR READOUT -------------------

//Helper functions to convert accel readings to real values

//Raw values to Acceleration (m/s^2)
ISM330::ISM330_VECTOR_TypeDef ISM330::readingToAccel(const uint8_t *readings) 
{
    int16_t rawX = (int16_t)(((uint16_t)readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)(((uint16_t)readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)(((uint16_t)readings[5] << 8) | readings[4]);

    float rawXfloat = static_cast<float>(rawX);
    float rawYfloat = static_cast<float>(rawY);
    float rawZfloat = static_cast<float>(rawZ);

    float x = ((rawXfloat-axBias) * accelConversion);
    float y = ((rawYfloat-ayBias) * accelConversion);
    float z = ((rawZfloat-azBias) * accelConversion);


    return {x, y, z};
}

//Raw to Gyro (Radians/second)
ISM330::ISM330_VECTOR_TypeDef ISM330::readingToGyro(const uint8_t *readings) 
{
    int16_t rawX = (int16_t)(((uint16_t)readings[1] << 8) | readings[0]);
    int16_t rawY = (int16_t)(((uint16_t)readings[3] << 8) | readings[2]);
    int16_t rawZ = (int16_t)(((uint16_t)readings[5] << 8) | readings[4]);

    float rawXfloat = static_cast<float>(rawX);
    float rawYfloat = static_cast<float>(rawY);
    float rawZfloat = static_cast<float>(rawZ);


    float x = ((rawXfloat-wxBias) * gyroConversion); 
    float y = ((rawYfloat-wyBias) * gyroConversion);
    float z = ((rawZfloat-wzBias) * gyroConversion);

    return {x, y, z};
}

// Actual Reading functions
ISM330::ISM330_VECTOR_TypeDef ISM330::readAccel() noexcept
{
    i2c.write(writeAddr, xReg, 1, false);
    i2c.read(readAddr, reinterpret_cast<char*>(xReadings), 6, true);
    
    ISM330_VECTOR_TypeDef accel_reading = readingToAccel(xReadings);
    return accel_reading;
}

ISM330::ISM330_VECTOR_TypeDef ISM330::readGyro() noexcept
{
    i2c.write(writeAddr, gReg, 1, false);
    i2c.read(readAddr, reinterpret_cast<char*>(gReadings), 6, true);

    ISM330_VECTOR_TypeDef gyro_reading = readingToGyro(gReadings);
    return gyro_reading;
}


// Combined Read function, should be more efficient because less I2C transactions
ISM330::ISM330_RAW_DATA_TypeDef ISM330::readAGraw() noexcept 
{
    i2c.write(writeAddr, gReg, 1, false); 
    i2c.read(readAddr, reinterpret_cast<char*>(agReadings), 12, true);

    int16_t rawGX = (int16_t)((agReadings[1] << 8) | agReadings[0]);
    int16_t rawGY = (int16_t)((agReadings[3] << 8) | agReadings[2]);
    int16_t rawGZ = (int16_t)((agReadings[5] << 8) | agReadings[4]);

    int16_t rawAX = (int16_t)((agReadings[7] << 8) | agReadings[6]);
    int16_t rawAY = (int16_t)((agReadings[9] << 8) | agReadings[8]);
    int16_t rawAZ = (int16_t)((agReadings[11] << 8) | agReadings[10]);

    return {rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ};
}



ISM330::ISM330_DATA_TypeDef ISM330::readAG() noexcept
{
    i2c.write(writeAddr, gReg, 1, false); 
    i2c.read(readAddr, reinterpret_cast<char*>(agReadings), 12, true);

    auto [gx,gy,gz] = readingToGyro(&agReadings[0]);
    auto [ax,ay,az] = readingToAccel(&agReadings[6]);

    ISM330_VECTOR_TypeDef accelVector = {ax, ay, az};
    ISM330_VECTOR_TypeDef gyroVector = {gx, gy, gz};

    return ISM330_DATA_TypeDef{ax, ay, az, gx, gy, gz};
}

void ISM330::getAGVectors(ISM330_VECTOR_TypeDef& accel, ISM330_VECTOR_TypeDef& gyro)
{
    ISM330_DATA_TypeDef imu_data = readAG();
    accel.x = imu_data.ax;
    accel.y = imu_data.ay;
    accel.z = imu_data.az;
    gyro.x = imu_data.gx;
    gyro.y = imu_data.gy;
    gyro.z = imu_data.gz;
}


// The original Mahony algorithm used a fast inverse square root approx, but for accuracy we can just do the real thing I think
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


IMU::EulerAngles ISM330::getImuAngles() {
    IMU::EulerAngles angles;
    computeAnglesMah(angles);
    return angles;
}

// Mahony Sensor fusion
void ISM330::mahonyStart(float prop_gain, float int_gain) {
    twoKp = 2 * prop_gain; // 2 * proportional gain (Kp)
    twoKi = 2 * int_gain;  // 2 * integral gain (Ki)
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
    // Use IMU algorithm if magnetometer measurement invalid
    // (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        mahonyUpdateIMU(dt);
        return;
    }
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

void ISM330::computeAnglesMah(IMU::EulerAngles& angles) {
    angles.roll = atan2f(mahq0 * mahq1 + mahq2 * mahq3, 0.5f - mahq1 * mahq1 - mahq2 * mahq2)*radToDeg;
    angles.pitch = asinf(-2.0f * (mahq1 * mahq3 - mahq0 * mahq2))*radToDeg;
    angles.yaw = atan2f(mahq1 * mahq2 + mahq0 * mahq3, 0.5f - mahq2 * mahq2 - mahq3 * mahq3)*radToDeg;
    grav[0] = 2.0f * (mahq1 * mahq3 - mahq0 * mahq2);
    grav[1] = 2.0f * (mahq0 * mahq1 + mahq2 * mahq3);
    grav[2] = 2.0f * (mahq0 * mahq0 - 0.5f + mahq3 * mahq3);
    anglesComputed = 1;
}

