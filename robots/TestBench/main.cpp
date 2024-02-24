#include "main.h"
#include <fstream>
#include <iostream>

// Initialize Motors
DJIMotor indexer(5, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_2, M3508);

// Initialize led's
DigitalOut led(L27);
DigitalOut led2(L26);

int SPEED_BOOST_VAL = 3600; // Unused currently
int SPEED_VAL = 6050;

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    int refLoop = 0;
    int serialLoop = 0;
    double lastShot = ext_shoot_data.data.bullet_speed;

    // Initialize time
    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    // Set PID Values
    double kP = 0.254;
    double kI = 0.146;
    double kD = 0.25;

    // Initialize Speed Values
    int speedVal = SPEED_VAL;
    int desiredSpeed = 0;
    int power = 0;

    // Set PID to system
    indexer.setPositionPID(kP, kI, kD);
    indexer.useAbsEncoder = false;

    // Setting up output of serial to external file
    ofstream outFile("heroSerialData.txt", ios::app);

    // Infinite loop
    while(true){
        timeStart = us_ticker_read();
        if ((timeStart - loopTimer)/ 1000 > 25){

            // Referee Loop Counter
            refLoop++;
            if (refLoop >= 25){
                refereeThread(&referee);
                refLoop = 0;
//                speedVal = SPEED_VAL;
            }

            // Blinking LED and Read Remote
            led2 =! led2;
            remoteRead();

            // Cuts Power if Remote Disconnects
            if (remote.leftSwitch() == Remote::SwitchState::UNKNOWN) {
                power = 0;
            } else {
                power = remote.leftX()*10;

            }

            // Sets Desired Speed for Serial Values
            desiredSpeed = ((int)remote.leftSwitch()-1) * speedVal;

            // Serial Loop to Print every 5 ms
            serialLoop++;
            if (serialLoop >= 5){
                serialLoop = 0;
                printff("%d %d\n", desiredSpeed, LFLYWHEEL.getData(VELOCITY));
            }

//            if (remote.leftX() > 10) {
//                speedVal = SPEED_BOOST_VAL;
//            }

            // Sets Speed
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