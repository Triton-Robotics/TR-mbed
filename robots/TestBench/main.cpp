#include "main.h"
#include <fstream>
#include <iostream>


DJIMotor indexer(5, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_2, M3508);

DigitalOut led(L27);
DigitalOut led2(L26);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    int refLoop = 0;
    int carsonLoop = 0;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    int pk = 10;
    int dk = 0.02;

    double kP = 0.254; // 0.428
    double kI = 0.146; // 0.434
    double kD = 0.127; // 0.214

    int speedVal = 7000;
    int desiredSpeed = 0;
    int power = 0;

    indexer.setPositionPID(kP, kI, kD);
    indexer.useAbsEncoder = false;

    // Setting up output of serial to external file
    ofstream outFile("heroSerialData.txt", ios::app);

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer)/ 1000 > 25){

            refLoop++;
            if (refLoop >= 25){
                refereeThread(&referee);
                refLoop = 0;

//                printff("%c%c\n", LFLYWHEEL.getData(VELOCITY) >> 8, (int8_t)LFLYWHEEL.getData(VELOCITY));
                //printff("%d %d\n", , LFLYWHEEL.getData(VELOCITY));
            }

            led2 =! led2;
            remoteRead();

            if (remote.leftSwitch() == Remote::SwitchState::UNKNOWN) {
                power = 0;
            } else {
                power = remote.leftX()*10;

            }

            desiredSpeed = ((int)remote.leftSwitch()-1) * speedVal;

            carsonLoop++;
            if (carsonLoop >= 5){
                carsonLoop = 0;

//                printff("%c%c\n", LFLYWHEEL.getData(VELOCITY) >> 8, (int8_t)LFLYWHEEL.getData(VELOCITY));
                printff("%d %d\n", desiredSpeed, LFLYWHEEL.getData(VELOCITY));
            }

            LFLYWHEEL.setSpeed(((int)remote.leftSwitch()-1) * -speedVal);
            RFLYWHEEL.setSpeed(((int)remote.leftSwitch()-1) * speedVal);
            indexer.setPower(power);

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }


        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}