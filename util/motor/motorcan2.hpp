#include "mbed.h"
#include "../algorithms/pid.hpp"
#include <cstdint>
<<<<<<< Updated upstream
#ifndef motor2_hpp
#define motor2_hpp
#include "motor.hpp"
#include "../communications/CANMsg.h"



class MotorCan2 : public Motor {
    public:
        MotorCan2(int canNum, motorType type = STANDARD, int ratio = 19, int inverted = false) : Motor(canNum,type,ratio,inverted) {MotorCan2::canOutput = 2;}
        //using Motor::Motor;
        //canOutput = 2;
=======
#ifndef motor_hpp
#define motor_hpp
#include "motor.hpp"
#include "../communications/CANMsg.h"

static CAN can1(PB_12,PB_13,1000000);

class MotorCan2 : Motor {


>>>>>>> Stashed changes
};
#endif