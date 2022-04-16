#include "mbed.h"
#include "../../util/motor/motor.hpp"
#include <cstdlib>
#ifndef chassis_subsystem_cpp
#define chassis_subsystem_cpp

enum chassisMode{TANK, MECANUM, BEYBLADE, RIGHT_ANGLE_MECANUM};

class ChassisSubsystem{
    public:
        int x;
        int y;
        //Motor motor1;

        ChassisSubsystem(int parameter1, double para2){
            //do something here
        }

        //add methods to make it easy for an end-user to use chassis without worry.
};

#endif