#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

DJIMotor testMot(5, CANHandler::CANBUS_1, GM6020, "testbench_motor");

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    testMot.setSpeedPID(0.5,0,0);

    bool prevL = false;
    bool switL = false;

    int motorSpeed = 0;

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
            }
            prevL = switL;
            remoteRead();
            switL = (remote.leftSwitch() == Remote::SwitchState::UP);

            // testMot.setPower(remote.leftX() * 3);
            // if(!prevL && switL){
            //     motorSpeed += 10;
            // }

            motorSpeed = remote.leftX() / 6;

            testMot.setSpeed(motorSpeed);
            int dir = 0;
            if(motorSpeed > 0){
                dir = 1;
            }else if(motorSpeed < 0){
                dir = -1;
            }
            testMot.pidSpeed.feedForward = dir * (874 + (73.7 * abs(motorSpeed)) + (0.0948 * motorSpeed * motorSpeed));
            printff("%d\t%d\t%d\n", testMot>>POWEROUT, motorSpeed, testMot>>VELOCITY);

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}