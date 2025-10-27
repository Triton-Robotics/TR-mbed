/*!
 *  @file Adafruit_ISM330DHCX.h
 *
 * 	I2C Driver for the Adafruit ISM330DHCX 6-DoF Accelerometer and Gyroscope
 *library
 *
 * 	This is a library for the Adafruit ISM330DHCX breakout:
 * 	https://www.adafruit.com/products/4480
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef ISM330_H_
#define ISM330_H_

#include "mbed.h"


#define ISM330_CHIP_ID 0x6B ///< ISM330 default device id from WHOAMI
#define ISM330_Who_Am_I 0x0F

// Gyroscope Output Registers

#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

//Accelerometer Output Registers

#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

class ISM330 {
public: 
    ISM330(SPI &spi, PinName csPin);

    //set up
    bool begin();
    uint8_t whoAmI();
    void reset();

    //Sensor Data
    void readAccel(float *x, float *y, float *z);
    void readGyro(float *x, float *y, float *z);

private: 
    uint8_t readRegister(uint8_t reg);
    void readMultiple(uint8_t reg, uint8_t *buf, uint8_t len);
    void writeRegister(uint8_t reg, uint8_t value);

    SPI &_spi;
    DigitalOut _cs;

};

/// Data Type Definitions? ///

// typedef struct {
//     uint8_t  chip_id;
//     uint8_t  acc_id;
//     uint8_t  mag_id;
//     uint8_t  gyr_id;
//     uint8_t  bootldr_rev_id;
//     uint16_t sw_rev_id;
// } ISM330_ID_INF_TypeDef;

// typedef struct {
//     double h;
//     double r;
//     double p;
// } ISM330_EULER_TypeDef;

// typedef struct {
//     double x;
//     double y;
//     double z;
//     double w;
// } ISM330_QUATERNION_TypeDef;

// typedef struct{
//     double yaw;
//     double roll;
//     double pitch;
// } ISM330_ANGULAR_POSITION_typedef;

// typedef struct {
//     double x;
//     double y;
//     double z;
// } ISM330_VECTOR_TypeDef;

// typedef struct {
//     int8_t acc_chip;
//     int8_t gyr_chip;
// } ISM330_TEMPERATURE_TypeDef;

#endif // ISM330_H_