#include "mbed.h"
#include "../src/main.hpp"

CANMotor chassis1(6,CANHandler::CANBUS_1,GIMBLY);

int maxspeed = 10000;

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

        while(1){

            if(lS == 1)
                chassis1.setPosition(lX * 100);
            else if(lS == 2)
                chassis1.setPower(2*lX);
            else if(lS == 3)
                chassis1.setSpeed(lX);

            CANMotor::printChunk(CANHandler::CANBUS_1,2);

            if(rS == 1)
                chassis1.multiTurn = 0;
            //chassis2.setSpeed(lY);
            //CANMotor::tick();
            //remotePrint();
            //printf("actual: %d desired: %d\n", chassis1.getData(ANGLE), (int)myremote.getStickData(LEFTJOYX, 0, maxspeed));

        }

}

