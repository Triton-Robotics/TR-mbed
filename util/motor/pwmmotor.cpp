#include "mbed.h"

/**
    * @brief Construct a new PWM Output 
    * @param pin 
    * @param defaultVal integerfrom 0 - 180 corresponds with a pulse width from 1ms to 2ms which is default for most servos/pwm motor controllers
    */
class PWMMotor : PwmOut{

    private:
        float intToPulse(int val) {
            return .001 + val * .001/180;
        }         

    public: 
        PWMMotor(PinName pin, int defaultVal = 0) : PwmOut(pin) {
            period(.02);
            pulsewidth(defaultVal);
        }            

        /**
        * @brief Set PWM Output 
        * @param motorVal integer from 0 - 180 which corresponds to a pulse width from 1ms to 2ms
        */
        void set(int motorVal) {
            pulsewidth(intToPulse(motorVal));
        }

};