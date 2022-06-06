#include "mbed.h"
#include "../src/main.hpp"

DigitalInOut clawPin(PA_13);
DigitalInOut rfidPin(PA_14);

NewChassisSubsystem chassis(4,2,1,3, CANHandler::CANBUS_1, C620);

CANMotor gimbalX(3,CANHandler::CANBUS_1,M3508);
CANMotor gimbalY(6,CANHandler::CANBUS_1,GM6020);

PWMMotor leftFlywheel(PA_5);
PWMMotor rightFlywheel(PA_6);

CANMotor indexer(7,CANHandler::CANBUS_1,M2006);

int maxspeed = 500;

// main() runs in its own thread in the OS

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);
    clawPin.output();
    rfidPin.output();
    //Motor::setCANHandler(&canPorts);
    //chassis1.pidSpeed.setPID(.1, 0, 0);
    //chassis1.pidPosition.setPID(.08,0,0.0125);
    indexer.outCap = 7000;

        while(1){

            if(rS == 2){
                //chassis1.setSpeed(lX);
                //chassis2.setSpeed(lX);
            }else{
                //chassis1.setPower(lX);
                //chassis2.setPower(lX);
            }
            chassis.movePow(lY,lX,Wh);

            //gimbalX.setPosition(rX);
            //gimbalY.setPosition(rY);

            gimbalX.setPower(rX * 2);
            gimbalY.setPower(rY * 6);

            if(lS == 2){
                clawPin = 0;
                rfidPin = 0;
            }else if(lS == 3){
                clawPin = 1;
                rfidPin = 0;
            }else if(lS == 1){
                clawPin = 0;
                rfidPin = 1;
            }
            
            int indexJamTime = 0;
            if(rS == 2){
                leftFlywheel.set(0);
                rightFlywheel.set(0); 
                indexer.setPower(0);
            }else if(rS == 3){
                leftFlywheel.set(60);
                rightFlywheel.set(60); 
                if(abs(indexer.getData(TORQUE)) > 1000 & abs(indexer.getData(VELOCITY)) < 20){ //jam
                    indexJamTime = us_ticker_read() /1000;
                }
                if(us_ticker_read() / 1000 - indexJamTime < 500){
                    indexer.setPower(-7000); //jam
                    printf("JAMMMMM- ");
                }else if(us_ticker_read() / 1000 - indexJamTime < 750){
                    indexer.setPower(7000); //jam
                    printf("POWER FORWARD- ");
                }else{
                    indexer.setPower(1700);
                }
                printf("AUTO-PWR:%d Jam-Free:%dms TORQ:%d, VELO:%d\n",indexer.powerOut,us_ticker_read() / 1000 - indexJamTime, indexer.getData(TORQUE), indexer.getData(VELOCITY));
            }else if(rS == 1){
                leftFlywheel.set(60);
                rightFlywheel.set(60); 
                indexer.setPower(rY*4);
                printf("MANUAL-PWR:%d VELO:%d\n", indexer.powerOut, indexer.getData(VELOCITY));
            }

            //chassis2.setSpeed(lY);
            //CANMotor::tick();
            //remotePrint();
        }

}


