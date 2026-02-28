#include "mbed.h"
#include <cstdint>
#include "LIS3MDL.h"

static constexpr uint8_t LIS3MDL_CHIP_ID = 0x3D;   // Expected WHO_AM_I
const char WhoAmIReg[] = {0x0F};

//I2C Reading/Writing Addresses
const int readAddr = 0b00111001;
const int writeAddr = 0b00111000;

// Initialization Commands
const char SWRST[] = {0x21,0x04}; // Software Reset command
const char fourGauss[] = {0x21, 0x00}; //

// Setting ODR Rates
const char lowPowerEnable[] = {0x20, 0x12}; // Low Power but 1khz
const char mediumPerformanceEnable[] = {0x20, 0x32}; // 560hz
const char highPerformanceEnable[] = {0x20, 0x52}; // 300hz
const char ultraHighPerformanceEnable[] = {0x20, 0x72}; // Most Accurate but only 155HZ

// Magnetometer Reading Registers
const char magReg[] = {0x28,0x29,0x2A,0x2B,0x2C,0x2D}; //Magnetometer Register values
// X_L, X_H, Y_L, Y_H, Z_L, Z_H


// Constructor
LIS3MDL::LIS3MDL(I2C &i2c, uint8_t address)  //Put in an i2c object, and the device address (in this case 0x3D)
        : i2c(i2c), _address(address)
    {
    }


// Begin function

bool LIS3MDL::begin() noexcept
{
    i2c.frequency(100000);

    i2c.write(writeAddr, SWRST, 2, false); 
    ThisThread::sleep_for(100ms);

    i2c.write(writeAddr, )

    i2c.write(writeAddr, lowPowerEnable, 2, false);

    i2c.write(writeAddr, WhoAmIReg, 1, false);
    char whoAmIReading;
    i2c.read(readAddr, reinterpret_cast<char*>(&whoAmIReading), 1, true);

    printf("WHO_AM_I: 0x%02X\r\n", whoAmIReading);
    return true;
}

float magConversionFactor = 1/6842.0; // For 4 Gauss its 6842 LSB/Gauss, 8 is 3421, 12 is 2281, 16 is 1711, so we inverse it 

std::tuple<float, float, float> LIS3MDL::readMagnetometer() noexcept
{
    uint8_t magbuffer[6]; 
    i2c.write(writeAddr, magReg, 1, false);
    i2c.read(readAddr, reinterpret_cast<char*>(magbuffer), 6, true);

    int16_t rawX = (int16_t)((magbuffer[1] << 8) | magbuffer[0]);
    int16_t rawY = (int16_t)((magbuffer[3] << 8) | magbuffer[2]);
    int16_t rawZ = (int16_t)((magbuffer[5] << 8) | magbuffer[4]);

    float x = rawX * magConversionFactor; // Conversion factor for LIS3MDL magnetometer
    float y = rawY * magConversionFactor;
    float z = rawZ * magConversionFactor;

    return std::make_tuple(x, y, z);
}

void LIS3MDL::getMagVector(LIS3MDL_VECTOR_TypeDef& magVector) {
    auto [mx, my, mz] = readMagnetometer();
        magVector.x = mx;
        magVector.y = my;
        magVector.z = mz;
}

