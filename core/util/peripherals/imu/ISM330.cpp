#include "mbed.h"
#include "ISM330.h"
#include <cstdint>

static constexpr int ISM330_CHIP_ID = 0x6b; ///< ISM330 default device id from WHOAMI
static constexpr int ISM330_WHO_AM_I = 0x0f;

// Provide storage for scale values in case they're treated as static by callers
// (some build units or usages can end up referencing class-qualified symbols).
// If you intend per-instance scales, remove these and ensure no code uses
// `ISM330::accelScale`/`ISM330::gyroScale` qualified names.
float ISM330::accelScale = 0.061f;
float ISM330::gyroScale = 17.50f;

ISM330::ISM330(SPI &spi, PinName csPin) noexcept
    : _spi(spi), _cs(csPin)
{
    _cs = 1; // deselect by default
    accelScale = 2;
    gyroScale = 2;
    // NOTE: csPin seems to be redundant
}

bool ISM330::begin() noexcept{
    _cs = 1;
    // Configure SPI: Mbed uses format/frequency rather than begin().
    // Adjust mode/clock as required by your hardware/device.
    _spi.format(8, 3); // 8-bit, mode 3 (CPOL=1, CPHA=1)
    _spi.frequency(100000); 


    return whoAmI() == ISM330_CHIP_ID;
}

uint8_t ISM330::whoAmI() noexcept{
    return readRegister(ISM330_WHO_AM_I);
}

void ISM330::reset() noexcept{
    writeRegister(0x12, 0x01); //0x12 is CTRL 3, we're telling it to soft reset
    thread_sleep_for(100); // possible change with polling perhaps?
}



uint8_t ISM330::readRegister(uint8_t reg) noexcept {
    _cs = 0;
    // Send register address (set MSB for read)
    _spi.write(reg | 0x80);
    // Read response
    uint8_t value = static_cast<uint8_t>(_spi.write(0x00));
    _cs = 1;
    return value;
}

// void ISM330::readMultiple(uint8_t reg, uint8_t *buf, uint8_t len) noexcept {
//     //digitalWrite(_csPin, LOW);
//     //checking if data is ready before we read
//     if (readRegister(STATUS_REG) & 0x03) {
//         _cs = 0;
//         _spi.write(reg | 0x80);
//         for (uint8_t i = 0; i < len; i++) {
//             buf[i] = static_cast<uint8_t>(_spi.write(0x00));
//             } 
//         _cs = 1; 
//     /*  Tried implementing this more robust way of reading bytes, I couldn't get it to work just yet     
//         uint8_t regReadFlag = reg | 0xC0; 
//         _spi.write(&regReadFlag, 1, *buf, len); */
//         }
//     }
    
void ISM330::readMultiple(uint8_t reg, uint8_t *buf, uint8_t len) noexcept {
    uint8_t regAddr = reg | 0x80; // MSB=1 for read + auto-increment
    _cs = 0;
    // Mbed SPI::write expects (const char*, int, char*, int).
    // Cast the uint8_t buffers to the required pointer types for a single transaction.
    _spi.write(reinterpret_cast<const char*>(&regAddr), 1, reinterpret_cast<char*>(buf), len); // single SPI transaction
    _cs = 1;
}

void ISM330::writeRegister(uint8_t reg, uint8_t value) noexcept {
    //digitalWrite(_csPin, LOW);
    _cs = 0;
        _spi.write(reg & 0x7F); // Clear MSB for write
        _spi.write(value);
    _cs = 1;
    //digitalWrite(_csPin, HIGH);
}


/*Accel scale values 0.122 (4G), 0.244 (8G), 0.488 (16G)
2G range: 0.061
4G range: 0.122
8G range: 0.244
16G range: 0.488
*/

void ISM330::setAccelRange(int r) noexcept {
    /*Taking current value and making a bit mask    
    which makes it so all values stay the same except bits 3 and 4 (control range), 
    which are set to zero for easy modification
    */
    uint8_t regVal = readRegister(CTRL1_XL);

    regVal &= 0b11110011;
    
    switch (r)
    {
    case 4 :
        regVal |= 0b10 << 2;
        accelScale = 0.122;
        break;
    
    case 8:
        regVal |= 0b11 << 2;
        accelScale = 0.244;
        break;

    case 16:
        regVal |= 0b01 << 2;
        accelScale = 0.488;
        break;

    default: //Default is 2G, which is 00, so we don't need to modify regVal, only confirm accelScale is 0.061
        accelScale = 0.061;
        break;
    }
    regVal |= 0b0111 << 4;
    writeRegister(CTRL1_XL, regVal);
}


void ISM330::readAccel(float *x, float *y, float *z) noexcept {
    uint8_t buf[6];
    //static constexpr float accelScale = 0.122;
    readMultiple(0x28, buf, 6);

    //Combining accel high and low 
    int16_t rawAX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawAY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawAZ = (int16_t)((buf[5] << 8) | buf[4]);

    // Later put in some kind of scaling if the XYZ isn't in proper units 

    *x = rawAX * accelScale;
    *y = rawAY * accelScale;
    *z = rawAZ * accelScale;
    
}


/* Gyro Scalevalues for range (degrees per second), chose 17.50 for now since 45RPM (270DPS) is the safe max
125 DPS: 4.375
250 DPS: 8.75
500 DPS: 17.50
1000 DPS: 35
2000 DPS: 70
*/



void ISM330::setGyroRange(int r) noexcept {
    /*Taking registar current value and making a bit mask to save current values apart from    
    bits 2,3 and 4 (2 enables the gyroscope, 3 and 4 set range), which are set to zero for easy modification
    */
    uint8_t regVal = readRegister(CTRL2_G);
    regVal &= 0b11110001;
    //Default for now is 500DPS since that's the safe max
    switch (r)
    {
    case 250:
        // regVal for 250 is 00, which is already what regVal is;
        gyroScale = 8.75;
        break;

    case 500:
        regVal |= 0b01 << 2;
        gyroScale = 17.50;
        break;

    case 1000:
        regVal |= 0b10 << 2;
        gyroScale = 35;
        break;

    case 2000:
        regVal |= 0b11 << 2;
        gyroScale = 70;
        break;
    
    default: //Default is 500 for now since ~270DPS is the safe max for our machines
        regVal |= 0b01 << 2;
        gyroScale = 17.50;
        break;
    }
    
    regVal |= 0b10; //Makes sure that bit 2 is on, since it's off by default
    writeRegister(CTRL2_G, regVal);
}

void ISM330::readGyro(float *x, float *y, float *z) noexcept {
    uint8_t buf[6];
    static constexpr float gyroScale = 17.50; // Currently unused, idk if its needed 
    readMultiple(0x22, buf, 6);

    //Combining high and low gyro values 
    int16_t rawGX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawGY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawGZ = (int16_t)((buf[5] << 8) | buf[4]);
}

std::tuple<float, float, float> ISM330::getAccel() noexcept
{
    uint8_t buf[6];
    //static constexpr float accelScale = 0.122;
    readMultiple(0x28, buf, 6);
    return std::make_tuple(
        static_cast<int16_t>(buf[1] << 8 | buf[0]) * accelScale,
        static_cast<int16_t>(buf[3] << 8 | buf[2]) * accelScale,
        static_cast<int16_t>(buf[5] << 8 | buf[4]) * accelScale
    );
}



std::tuple<float, float, float> ISM330::getGyro() noexcept {
    uint8_t buf[6];
    //static constexpr float gyroScale = 17.50; 
    readMultiple(0x22, buf, 6);

    return std::make_tuple(
        static_cast<int16_t>((buf[1] << 8) | buf[0]) * gyroScale,
        static_cast<int16_t>((buf[3] << 8) | buf[2]) * gyroScale,
        static_cast<int16_t>((buf[5] << 8) | buf[4]) * gyroScale
    );
}

