#include "mbed.h"
#include "../src/main.hpp"

//Thread remote(osPriorityHigh);

CANMotor chassis1(3,CANHandler::CANBUS_1,M3508);

int maxspeed = 10000;

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

        while(1){
            //chassis1.getFeedback(1);
            // printf("multiturn:%d\n", chassis1.getData(MULTITURNANGLE));
            // chassis1.printAllMotorData();
            if(lS == 1)
                chassis1.setPosition(lX * 100);
            else if(lS == 2)
                chassis1.setPower(2*lX);
            else if(lS == 3)
                chassis1.setSpeed(lX);

            if(rS == 1)
                chassis1.multiTurn = 0;
            //chassis2.setSpeed(lY);
            //CANMotor::tick();
            remotePrint();
            //printf("actual: %d desired: %d\n", chassis1.getData(ANGLE), (int)myremote.getStickData(LEFTJOYX, 0, maxspeed));

        }

}

