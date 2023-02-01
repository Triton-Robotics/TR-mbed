//
// Created by ankit on 1/31/23.
//

#include "PWMMotor.h"

float PWMMotor::intToPulse(int val) {
    return .001 + val * .001/180;
}

/**
        * @brief Constructor for PWMMotor
        * @param defaultVal some motors will require a default value to send to the motor to initialize it.
        */
PWMMotor::PWMMotor(PinName pin, int defaultVal) : PwmOut(pin) {
        period(.02);
        pulsewidth(intToPulse(defaultVal));
}

/**
* @brief Set PWM Output
* @param motorVal integer from 0 - 180 which corresponds to a pulse width from 1ms to 2ms
*/
void PWMMotor::set(int motorVal) {
    pulsewidth(intToPulse(motorVal));
}