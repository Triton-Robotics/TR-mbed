#include "main.h"
//#include "/Infantry/Infantry.h"
#include <cstdlib>
#include <commands/RamseteCommand.h>
#include <iostream>

#define PI 3.14159265

///#define LOWERBOUND 1000
//#define UPPERBOUND 2000

I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);
DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left
DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap

// RamseteCommand command(
//         Pose2D(0, 0, 0), Pose2D(20, 20, 0), 2, &chassis);
DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);



Thread imuThread;

// double getPitchAngle(geometry_msgs__msg__Vector3Stamped jetsonAngles) {
//     return asin(-jetsonAngles.vector.y);
// }
//
// double getYawAngle(geometry_msgs__msg__Vector3Stamped jetsonAngles) {
//     return atan2(jetsonAngles.vector.x, jetsonAngles.vector.z);
// }

void runImuThread()
{
    chassis.initializeImu();
    while (true)
    {
        chassis.readImu();
        ThisThread::sleep_for(25);
    }
}

void pitchSetPosition(){
    //int pitchSetPoint = (-remote.rightY() * 0.5) + 6200;
    int pitchSetPoint = 5960 - (5960 - 5350)/660.0 * remote.rightY();

    /* TODO: test min and max pitch position */
    // low
    if(pitchSetPoint > 6570)
        pitchSetPoint = 6570;
    // high
    else if(pitchSetPoint < 5350)
        pitchSetPoint = 5350;

    pitch.setPosition(pitchSetPoint);
    //printff("%d.%d %d\n", pitchSetPoint, pitch.getData(ANGLE), pitch.powerOut);
}



int main(){

    imuThread.start(runImuThread);
    // float speedmultiplier = 3;
    // float powmultiplier = 2;
    // float translationalmultiplier = 1.5; // was 3
    // float beybladespeedmult = 1;

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);

    pitch.setPositionPID(18, 0.01, 850); //12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
    //pitch.setPositionPID(22, 0.0000000001, 250); //12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
    pitch.setPositionIntegralCap(6000);
    //pitch.setPositionOutputCap(6000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;


    //command.initialize();

    unsigned long loopTimer = us_ticker_read();

    int refLoop = 0;
    double rotationalPower = 0;
    DJIMotor::s_getFeedback();

    int counter = 0;

    unsigned long lastTime = 0;

    while (true){
        unsigned long timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            led = !led;
            loopTimer = timeStart;
            remoteRead();
            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2= !led2;
                printff("%d %d %lf\n", pitch2>>POWEROUT, pitch>>POWEROUT, pitch.pidPosition.DPower());
            }


            pitch2.setPosition(3400);
            /**
             * rightSwitch controls
             */
            if (remote.rightSwitch() == Remote::SwitchState::UP){          // All non-serializer motors activated
                led3 = 1;
                unsigned long time = us_ticker_read();
                lastTime = time;
                pitchSetPosition();
            } else if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN){ // disable all the non-serializer components
                pitch.setPower(0);
            } else if (remote.rightSwitch() == Remote::SwitchState::DOWN){           // beyblade mode
                unsigned long time = us_ticker_read(); //time for pid
                double r = 4000;
                lastTime = time;
                pitchSetPosition();
            }



            DJIMotor::s_sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}