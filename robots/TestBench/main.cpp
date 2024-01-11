#include "main.h"
#include <fstream>
#include <iostream>


DJIMotor indexer(5, CANHandler::CANBUS_1, C610);
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_1, M3508);
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_1, M3508);

DigitalOut led(L27);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    int flywheelspd = 6500;
    int refLoop = 0;

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    int pk = 10;
    int dk = 0.02;

    double kP = 1.5;
    double kI = 0.0000021;
    double kD = 0.04;

    indexer.setPositionPID(1, 0, 0);
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

                printff("cP: %f\n", ext_power_heat_data.data.chassis_power);
            }

            led =! led;
            remoteRead();


            LFLYWHEEL.setSpeed(((int)remote.leftSwitch()-1) * -3200);
            RFLYWHEEL.setSpeed(((int)remote.leftSwitch()-1) * 3200);
            indexer.setPower(remote.leftX()*10);

            outFile << LFLYWHEEL.getData(VELOCITY) << endl;

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }


        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}