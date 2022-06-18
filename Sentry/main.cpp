#include "mbed.h"
#include "../src/main.hpp"


Thread remote(osPriorityHigh);
int gimYBound[2] = {32,96};
int gim2YBound[2] = {700,2300};
int gimXBound[2] = {-180,180};

CANMotor chassis1(3,CANHandler::CANBUS_1,M3508);
CANMotor chassis2(4,CANHandler::CANBUS_1,M3508);

CANMotor gimbalX(3,CANHandler::CANBUS_1,GM6020);
CANMotor gimbalY(6,CANHandler::CANBUS_1,GM6020);

PWMMotor leftFlywheelTop(PA_5);
PWMMotor rightFlywheelTop(PA_6);
PWMMotor leftFlywheelBot(PB_6);
PWMMotor rightFlywheelBot(PA_7);

CANMotor indexer(5,CANHandler::CANBUS_1,M2006);

int maxspeed = 500;

// main() runs in its own thread in the OS

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);
    //Motor::setCANHandler(&canPorts);
    chassis1.multiTurn = 0;
    chassis2.multiTurn = 0;
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

            //gimbalX.setPosition(rX);
            //gimbalY.setPosition(rY);

            //gimbalX.setPower(rX * 2);
            //gimbalY.setPower(rY * 6);

            if(lS == 1){
                leftFlywheelTop.set(60);
                leftFlywheelBot.set(60);
                rightFlywheelTop.set(60);
                rightFlywheelBot.set(60);   
            }else if(lS == 2){
                leftFlywheelTop.set(0);
                leftFlywheelBot.set(0);
                rightFlywheelTop.set(0);
                rightFlywheelBot.set(0);   
            }else if(lS == 3){
                leftFlywheelTop.set(lX * 60.0/1000);
                leftFlywheelBot.set(lY * 60.0/1000);
                rightFlywheelTop.set(rX * 60.0/1000);
                rightFlywheelBot.set(rY * 60.0/1000); 
            }
            
            int indexJamTime = 0;
            if(rS == 2){
                indexer.setPower(0);
            }else if(rS == 3){
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
                indexer.setPower(rY*4);
                printf("MANUAL-PWR:%d VELO:%d\n", indexer.powerOut, indexer.getData(VELOCITY));
            }

            //chassis2.setSpeed(lY);
            //CANMotor::tick();
            remotePrint();
        }

}

