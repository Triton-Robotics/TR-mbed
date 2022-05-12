#include "mbed.h"
#include "momsSpaghetti.hpp"
#include "../communications/causeSpaghettiComesOnceInALifetime.hpp"
#ifndef motorhandler_hpp
#define motorhandler_hpp

enum sendID{
    x200 = 0,
    x1ff = 1,
    x1FF = 1,
    x2ff = 2,
    x2FF = 2
};

class MotorHandler{
    public:
        NewCANHandler* canHandler1;
        NewCANHandler* canHandler2;

        CANMotor motors[3][4];

        MotorHandler(PinName can1Rx, PinName can1Tx, PinName can2Rx, PinName can2Tx)
        {
            canHandler1 = new NewCANHandler(can1Rx,can1Tx);
            canHandler2 = new NewCANHandler(can2Rx,can2Tx);
        }

        void positionPIDMove(PID* pid, int desiredVal, int actualVal, int dt)
        {

        }

        void speedPIDMove(PID* pid, int desiredVal, int actualVal, int dt)
        {

        }

        void move(int value)
        {
            
        }
};

#endif