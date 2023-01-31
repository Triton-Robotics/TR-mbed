//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_PWMMOTOR_H
#define TR_EMBEDDED_PWMMOTOR_H

#include "mbed.h"

class PWMMotor : PwmOut{

private:
    static float intToPulse(int val);

public:
    PWMMotor(PinName pin, int defaultVal = 0);
    void set(int motorVal);
};


#endif //TR_EMBEDDED_PWMMOTOR_H
