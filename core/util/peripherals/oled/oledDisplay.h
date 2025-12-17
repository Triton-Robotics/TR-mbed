#ifndef TR_EMBEDDED_OLEDDISPLAY_H
#define TR_EMBEDDED_OLEDDISPLAY_H

#include "mbed.h"
#include <cmath>
#include <string>
#include "Adafruit_SSD1306.h"
#include "util/motor/DJIMotor.h"
//we will be using Adafruit_SSD1306

//I2C i2c(PB_7,PB_8);
//Adafruit_SSD1306_I2c oled(i2c,NC);

class oledDisplay {

public:
    oledDisplay();
    I2C i2c;
    Adafruit_SSD1306_I2c oled;
    //Adafruit_SSD1306_I2c oled;
    //explicit oledDisplay(I2C &i2c);
    int motorCount();
};


#endif //TR_EMBEDDED_OLEDDISPLAY_H
