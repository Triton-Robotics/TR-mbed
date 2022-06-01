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

PWMMotor leftFlywheelTop(PA_5);
PWMMotor rightFlywheelTop(PA_6);
PWMMotor leftFlywheelBot(PB_6);
PWMMotor rightFlywheelBot(PA_7);

CANMotor indexer(2,CANHandler::CANBUS_1,M2006);

int maxspeed = 500;

// main() runs in its own thread in the OS

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);
    //Motor::setCANHandler(&canPorts);
    chassis1.multiTurn = 0;
    //chassis1.pidSpeed.setPID(.1, 0, 0);
    //chassis1.pidPosition.setPID(.08,0,0.0125);
    indexer.outCap = 2500;

        while(1){

            if(rS == 2){
                chassis1.setSpeed(lX);
                chassis2.setSpeed(lX);
            }else{
                chassis1.setPower(lX);
                chassis2.setPower(lX);
            }
            //gimbalX.setPosition(rX);
            //gimbalY.setPosition(rY);
            //gimbalX.setPower(rX);
            //gimbalY.setPower(rY);

            if(lS == 2){
                leftFlywheelTop.set(60);
                leftFlywheelBot.set(60);
                rightFlywheelTop.set(60);
                rightFlywheelBot.set(60);   
            }else{
                leftFlywheelTop.set(0);
                leftFlywheelBot.set(0);
                rightFlywheelTop.set(0);
                rightFlywheelBot.set(0);   
            }

            //indexer.setSpeed((rS - 2) * 700);
            
            //indexer.setPower((rS - 2) * 1400);
            
            int indexJamTime = 0;
            if(rS == 2){
                indexer.setPower(0);
            }else if(rS == 3){
                if(indexer.powerOut > abs(indexer.getData(VELOCITY)) < 20){ //jam
                    indexJamTime = us_ticker_read() /1000;
                }
                if(us_ticker_read() / 1000 - indexJamTime < 500){
                    indexer.setPower(-3000); //jam
                    printf("JAMMMMM- ");
                }else{
                    indexer.setPower(1700);
                }
                printf("PWR:%d Jam-Free:%dms TORQ:%d\n",indexer.powerOut,us_ticker_read() / 1000 - indexJamTime, indexer.getData(TORQUE));
            }else if(rS == 1){
                indexer.setPower(rY*4);
            }

            // if(rS == 2){
            //     indexer.setPower(0);
            // }else if(rS == 3){
            //     indexer.setPower(1500);
            // }else if(rS == 1){
            //     indexer.setPower(-2000);
            // }

            //chassis2.setSpeed(lY);
            //CANMotor::tick();
            //remotePrint();
        }

}

