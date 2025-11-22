#include "mbed.h"
#include "ISM330.h"
#include <cstdint>

static constexpr uint8_t ISM330_CHIP_ID = 0x6B;   // Expected WHO_AM_I
static constexpr uint8_t ISM330_WHO_AM_I = 0x0F;

// Accelerometer and gyro register definitions
static constexpr uint8_t CTRL1_XL = 0x10;
static constexpr uint8_t CTRL2_G  = 0x11;
static constexpr uint8_t CTRL3_C  = 0x12;
static constexpr uint8_t CTRL4_C  = 0x13;
static constexpr uint8_t STATUS_REG = 0x1E;
static constexpr uint8_t OUTX_L_G = 0x22;
static constexpr uint8_t OUTX_L_XL = 0x28;

static constexpr float accelScale = 0.122f;  // mg/LSB @ ±4g
static constexpr float gyroScale  = 70.0f;   // mdps/LSB @ ±2000 dps


// ------------------- CONSTRUCTOR -------------------
ISM330::ISM330(SPI &spi, PinName csPin) noexcept
    : _spi{spi}, _cs{csPin}
{
    _cs = 1; // Deselect by default
}


// ------------------- INITIALIZATION -------------------
bool ISM330::begin() noexcept
{
    _spi.format(8, 3);                 // Using mode 3 since that's what the libraries do
    _spi.frequency(1'000'000);
    _cs = 1; ThisThread::sleep_for(3ms);

    uint8_t id = whoAmI();
    printf("WHO_AM_I: 0x%02X\r\n", id);
    if (id != 0x6B) return false;      // DHCX id

    writeRegister(CTRL3_C, 0x01);      // SW_RESET
    ThisThread::sleep_for(20ms);

    // User bank
    // writeRegister(0x01, 0x00);

    // BDU=1, IF_INC=1
    writeRegister(CTRL3_C, 0x44);

    //Setting SPI MODE CTRL4_C 
    writeRegister(19, 0x04);

    // Accel: 104 Hz, ±4g, BW=100 Hz -> 0x4A CTRL1_XL
    writeRegister(16, 0x4A); //0b0100'1010
    // Gyro: 104 Hz, 2000 dps        -> 0x4C
    writeRegister(17, 0x4C); //0b0100'1100 CTRL2_G 
    ThisThread::sleep_for(20ms);

    printf("CTRL3_C=0x%02X, CTRL1_XL=0x%02X, CTRL2_G=0x%02X\n",
           readRegister(CTRL3_C), readRegister(CTRL1_XL), readRegister(CTRL2_G));

    return true;
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

    _spi.write(reg | 0x80);
    uint8_t value = static_cast<uint8_t>(_spi.write(0x00));
    _cs = 1;
    return value;
    
    // uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0x00 };
    // uint8_t rx[2] = { 0, 0 };
    // wait_ns(50);
    // _spi.write(reinterpret_cast<const char*>(tx), 2, reinterpret_cast<char*>(rx), 2);
    // wait_ns(50);
    // _cs = 1;
    // return rx[1];
}

void ISM330::readMultiple(uint8_t reg, uint8_t* buf, uint8_t len) noexcept
{
    uint8_t addr = static_cast<uint8_t>(reg | 0x80); // R=1, auto-inc=1
    _cs = 0;
    wait_ns(50);
    _spi.write(reinterpret_cast<const char*>(&addr), 1, nullptr, 0);
    _spi.write(nullptr, 0, reinterpret_cast<char*>(buf), len); // clock out N bytes
    wait_ns(50);
    _cs = 1;
}

// void ISM330::writeRegister(int reg, int value) noexcept
// {
//     // uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), value };
//     _cs = 0;
//     wait_ns(50); // tCSS setup
//     // Use the buffered form so both bytes go under one CS window
//     // _spi.write(reg);
//     // _spi.write(value);
    
//     //_spi.write(reinterpret_cast<const char*>(tx), 2, nullptr, 0);
//     _cs = 1;    
//}


// //Helper 'Callback' function for SPI transfer
// void ISM330::transferCallBack(int event) {
//     if (event & SPI_EVENT_COMPLETE) {
//         printf("SPI transfer complete!\n");
//     }
    
//     if (event & SPI_EVENT_ERROR) {
//         printf("SPI error occurred!\n");
//     }
// }

void ISM330::writeRegister(uint8_t reg, uint8_t value) noexcept
{    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), value };
     uint8_t rx[2];
_cs = 0;
// _spi.write(reg & 0x7F); // MSB=0 for write
// _spi.write(value);
//trying with transfer

_spi.transfer(
    tx,
    2,
    rx,
    2,
    [](int event){
        if (event & SPI_EVENT_COMPLETE) {
            printf("SPI transfer complete!\n");
        }
    
        if (event & SPI_EVENT_ERROR) {
            printf("SPI error occurred!\n");
        }
    }  );

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
