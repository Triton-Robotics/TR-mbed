#include "mbed.h"
#include "ISM330.h"

ISM330::ISM330(SPI &spi, PinName csPin)
    : _spi(spi), _cs(csPin)
{
    _cs = 1; // deselect by default
}



bool ISM330::begin() {
    _cs = 1;

    _spi.begin();

    uint8_t id = whoAmI();
    return (id == ISM330_CHIP_ID);
}

uint8_t ISM330::whoAmI() {
    return readRegister(ISM330_Who_Am_I);
}

void ISM330::reset() {
    writeRegister(0x12, 0x01); //0x12 is CTRL 3, we're telling it to soft reset
    thread_sleep_for(100);
}

uint8_t ISM330::readRegister(uint8_t reg) {
    _cs = 0;
    _spi.transfer(reg | 0x80); // Set MSB for read
    uint8_t value = _spi.transfer(0x00);
    //digitalWrite(_csPin, HIGH);
    _cs = 1;
    return value;
}

void ISM330::readMultiple(uint8_t reg, uint8_t *buf, uint8_t len) {
    //digitalWrite(_csPin, LOW);
    _cs = 0;
    _spi.transfer(reg | 0xC0);
        for (uint8_t i = 0; i < len; i++) {
        buf[i] = _spi.transfer(0x00);
    }
    _cs = 0;
    //digitalWrite(_csPin, HIGH);
}

void ISM330::writeRegister(uint8_t reg, uint8_t value) {
    //digitalWrite(_csPin, LOW);
    _cs = 0;
    _spi.transfer(reg & 0x7F); // Clear MSB for write
    _spi.transfer(value);
    _cs = 1;
    //digitalWrite(_csPin, HIGH);
}

float accelScale = 0.122 

/*Accel scale values 0.122 (4G), 0.244 (8G), 0.488 (16G)
2G range: 0.061
4G range: 0.122
8G range: 0.244
16G range: 0.488
*/

void ISM330::readAccel(float *x, float *y, float *z) {
    uint8_t buf[6];
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


float gyroScale = 17.50

/* Gyro Scalevalues for range (degrees per second), chose 17.50 for now since 45RPM (270DPS) is the safe max
125 DPS: 4.375
250 DPS: 8.75
500 DPS: 17.50
1000 DPS: 35
2000 DPS: 70
*/


void ISM330::readGyro(float *x, float *y, float *z) {
    uint8_t buf[6];
    readMultiple(0x22, buf, 6);

    //Combining high and low gyro values 
    int16_t rawGX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawGY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawGZ = (int16_t)((buf[5] << 8) | buf[4]);
}

