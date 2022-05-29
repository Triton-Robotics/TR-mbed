#include "mbed.h"
#include "../src/main.hpp"

Thread remote(osPriorityHigh);

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
            // chassis1.setPosition(myremote.getStickData(LEFTJOYX, 0, maxspeed));

        }

}

