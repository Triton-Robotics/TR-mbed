#ifndef LIS3MDL_H_
#define LIS3MDL_H_
#include "mbed.h"
#include <tuple>
#include <cstdint>

class LIS3MDL {
public:

    LIS3MDL(I2C &i2c, uint8_t address) noexcept;

    bool begin() noexcept;

    uint8_t whoAmI() noexcept;

    void reset() noexcept;

    //Structs
    typedef struct {
        float x;
        float y;
        float z;
    } LIS3MDL_VECTOR_TypeDef;

    //Magnetometer Reading functions
    std::tuple<float, float, float> readMagnetometer() noexcept;

    void getMagVector(LIS3MDL_VECTOR_TypeDef& magVector);



private:
    I2C &i2c;
    uint8_t _address;
};

#endif