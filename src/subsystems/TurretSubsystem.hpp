#include "mbed.h"
#include "../../util/motor/motor.hpp"
#include "../../util/motor/pwmmotor.cpp"
#include <cstdlib>
#include <algorithm>
#ifndef turret_subsystem_cpp
#define turret_subsystem_cpp

enum turretMode{RELATIVE, ANGULAR};

class TurretSubsystem{
    public:
        Motor* gimbalX;
        Motor* gimbalY;
        PWMMotor* flywheelL;
        PWMMotor* flywheelR;
        Motor* serializer;
        int xBounds[2] = {0,0}; //angle array for the x axis as a bound for the amount of allowed rotation;
        int yBounds[2] = {0,0}; //angle array for the y axis as a bound for the amount of allowed rotation;

        TurretSubsystem(Motor gX, Motor gY, PWMMotor flyL, PWMMotor flyR, Motor serialize)
        {
            gimbalX = &gX;
            gimbalY = &gY;
            flywheelL = &flyL;
            flywheelR = &flyR;
            serializer = &serialize;
        }

        void gimbalMove(int angleX, int angleY){
            angleX = std::min(std::max(xBounds[1],angleX),xBounds[0]);
            angleY = std::min(std::max(yBounds[1],angleY),yBounds[0]);
            gimbalX->setDesiredPos(angleX);
            gimbalY->setDesiredPos(angleY);
        }


};

#endif