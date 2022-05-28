#include "mbed.h"
#include "../src/main.hpp"


Thread remote(osPriorityHigh);
int gimYBound[2] = {32,96};
int gim2YBound[2] = {700,2300};
int gimXBound[2] = {-180,180};

CANMotor chassis1(3,CANHandler::CANBUS_1,M3508);
CANMotor chassis2(4,CANHandler::CANBUS_1,M3508);

CANMotor gimbalX(7,CANHandler::CANBUS_1,GM6020);
CANMotor gimbalY(6,CANHandler::CANBUS_1,GM6020);

PWMMotor leftFlywheel(PA_5);
PWMMotor rightFlywheel(PA_6);

int maxspeed = 500;

// main() runs in its own thread in the OS

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);
    //Motor::setCANHandler(&canPorts);


        while(1){

            chassis1.setPosition(myremote.getStickData(LEFTJOYX, 0, maxspeed));
            // chassis2.setSpeed(myremote.getStickData(LEFTJOYX, 0, maxspeed));
            CANMotor::tick();
        }

}

