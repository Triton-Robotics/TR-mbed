#include "main.h"
#include "subsystems/ChassisSubsystem.h"

// #include "algorithms/eigen-3.4.0/Eigen/Dense"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

DJIMotor testMot(4, CANHandler::CANBUS_1, M3508, "testbench_motor");

int main()
{

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();
    DJIMotor::initializedWarning = false;

    Chassis.setYawReference(&yawOne, 7167); // "7167" is the number of ticks of yawOne considered to be robot-front

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    testMot.setSpeedPID(1.5,0,0);

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;

            refLoop++;
            if (refLoop >= 10)
            {
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
                printff("datum:%d %d %d\n", testMot>>ANGLE, testMot>>VELOCITY, remote.leftX());
            }

            remoteRead();

            testMot.setPower(remote.leftX() * 3);
            
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
