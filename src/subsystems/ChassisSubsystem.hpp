#include "mbed.h"
#include "../../util/motor/motor.hpp"
#include <cstdlib>
#ifndef chassis_subsystem_cpp
#define chassis_subsystem_cpp

enum chassisMode{TANK, MECANUM, BEYBLADE, RIGHT_ANGLE_MECANUM};

class ChassisSubsystem{
    public:
        Motor MotorRightFront;
        Motor MotorLeftFront;
        Motor MotorRightBack;
        Motor MotorLeftBack;
       
       ChassisSubsystem(size_t m1_pin,size_t m2_pin,size_t m3_pin,size_t m4_pin, CANHandler::CANBus MOTOR,motorType TYPE):
        MotorRightFront(m1_pin,MOTOR,TYPE),
        MotorLeftFront(m2_pin,MOTOR,TYPE),
        MotorRightBack(m3_pin,MOTOR,TYPE),
        MotorLeftBack(m4_pin,MOTOR,TYPE)
        {}
       
        
        void set_speeds(int RF,int LF, int RB, int LB){

            MotorRightFront.setDesiredSpeed(RF);
            MotorLeftFront.setDesiredSpeed(LF);
            MotorRightBack.setDesiredSpeed(RB);
            MotorLeftBack.setDesiredSpeed(LB);
            
        }

        void set_Direction(int y, int x, int rx){
            y=-y/660;
            x=x/660;
            rx = rx/660;
            int RF = y - x - rx, LF = y + x + rx, RB = y + x - rx, LB = y - x + rx;
            set_speeds(RF,LF,RB,LB);
            Motor::tick();
        }

};

#endif  
