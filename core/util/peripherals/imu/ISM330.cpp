#include "mbed.h"
#include "ISM330.h"
#include <cstdint>

static constexpr uint8_t ISM330_CHIP_ID = 0x6B;   // Expected WHO_AM_I
static constexpr uint8_t ISM330_WHO_AM_I = 0x0F;

// Accelerometer and gyro register definitions
static constexpr uint8_t CTRL1_XL = 0x10;
static constexpr uint8_t CTRL2_G  = 0x11;
static constexpr uint8_t CTRL3_C  = 0x12;
static constexpr uint8_t STATUS_REG = 0x1E;
static constexpr uint8_t OUTX_L_G = 0x22;
static constexpr uint8_t OUTX_L_XL = 0x28;

static constexpr float accelScale = 0.122f;  // mg/LSB @ ±4g
static constexpr float gyroScale  = 70.0f;   // mdps/LSB @ ±2000 dps


// ------------------- CONSTRUCTOR -------------------
ISM330::ISM330(SPI &spi, PinName csPin) noexcept
    : _spi(spi), _cs(csPin)
{
    _cs = 1; // Deselect by default
}


// ------------------- INITIALIZATION -------------------
bool ISM330::begin() noexcept
{
    _cs = 1;
    _spi.format(8, 3);          // ✅ ISM330 requires mode 3 (CPOL=1, CPHA=1)
    _spi.frequency(1'000'000);

    // --- Software reset ---
    writeRegister(CTRL3_C, 0x01);
    ThisThread::sleep_for(5ms);
    while (readRegister(CTRL3_C) & 0x01) {
        ThisThread::sleep_for(1ms);
    }

    // --- IF_INC + BDU ---
    uint8_t ctrl3 = readRegister(CTRL3_C);
    ctrl3 |= (1 << 2) | (1 << 6);
    writeRegister(CTRL3_C, ctrl3);

    // --- Enable accelerometer ---
    // ODR_XL=0b0100 (104 Hz), FS_XL=0b10 (±4g),
    // BW0_XL=1 (LPF enable) -> bits [1:0] = 10 for 100 Hz LPF
    writeRegister(CTRL1_XL, 0x4A);   // ✅ 0b0100_1010

    // --- Enable gyro ---
    // ODR_G=0b0100 (104 Hz), FS_G=0b11 (2000 dps)
    writeRegister(CTRL2_G, 0x4C);    // ✅ same as before, correct bits

    // --- Wait for sensor to start producing data ---
    ThisThread::sleep_for(20ms);

    // --- Verify device ID ---
    uint8_t id = whoAmI();
    uint8_t xl = readRegister(CTRL1_XL);
    uint8_t g  = readRegister(CTRL2_G);
    printf("CTRL1_XL=0x%02X, CTRL2_G=0x%02X\n", xl, g);
    printf("id check: %d", id == ISM330_CHIP_ID);
    return id == ISM330_CHIP_ID;
}


// ------------------- BASIC REGISTER ACCESS -------------------
uint8_t ISM330::whoAmI() noexcept
{
    return readRegister(ISM330_WHO_AM_I);
}

void ISM330::reset() noexcept
{
    writeRegister(CTRL3_C, 0x01);
    ThisThread::sleep_for(100ms);
}

uint8_t ISM330::readRegister(uint8_t reg) noexcept
{
    _cs = 0;
    _spi.write(reg | 0x80);  // MSB=1 for read
    uint8_t value = _spi.write(0x00);
    _cs = 1;
    return value;
}

void ISM330::readMultiple(uint8_t reg, uint8_t *buf, uint8_t len) noexcept
{
    uint8_t addr = reg | 0xC0;  // Read + auto-increment
    _cs = 0;
    _spi.write(addr);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = _spi.write(0x00);
    }
    _cs = 1;
}

void ISM330::writeRegister(uint8_t reg, uint8_t value) noexcept
{
    _cs = 0;
    _spi.write(reg & 0x7F); // MSB=0 for write
    _spi.write(value);
    _cs = 1;
}


// ------------------- SENSOR READOUT -------------------
void ISM330::readAccel(float *x, float *y, float *z) noexcept
{
    uint8_t status = readRegister(STATUS_REG);
    if (!(status & 0x01)) {
        *x = *y = *z = 0.0f;  // accel not ready
        return;
    }

    uint8_t buf[6];
    readMultiple(OUTX_L_XL, buf, 6);

    int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);

    constexpr float g = 9.80665f;
    *x = rawX * accelScale * g / 1000.0f;
    *y = rawY * accelScale * g / 1000.0f;
    *z = rawZ * accelScale * g / 1000.0f;
}


void ISM330::readGyro(float *x, float *y, float *z) noexcept
{
    uint8_t status = readRegister(STATUS_REG);
    if (!(status & 0x02)) {
        *x = *y = *z = 0.0f;  // gyro not ready
        return;
    }

    uint8_t buf[6];
    readMultiple(OUTX_L_G, buf, 6);

    int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);

    constexpr float DPS_TO_RAD = 3.14159265358979323846f / 180.0f;
    *x = rawX * gyroScale * DPS_TO_RAD / 1000.0f;
    *y = rawY * gyroScale * DPS_TO_RAD / 1000.0f;
    *z = rawZ * gyroScale * DPS_TO_RAD / 1000.0f;
}


// ------------------- TUPLE HELPERS -------------------
std::tuple<float, float, float> ISM330::getAccel() noexcept
{
    float x, y, z;
    readAccel(&x, &y, &z);
    return std::make_tuple(x, y, z);
}

std::tuple<float, float, float> ISM330::getGyro() noexcept
{
    float x, y, z;
    readGyro(&x, &y, &z);
    return std::make_tuple(x, y, z);
}
