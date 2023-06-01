#include "main.h"
#include "mbed.h"

#define dr (8191.0 / 9.0)
#define smooth true

/*
 * if smooth is true, balls will smoothly come out of serializer with no double feed,
 * but has a low chance of getting stuck around 1/50
 *
 * if smooth is false, balls will come out of serializer with a higher chance of double feed,
 * but there is virtually no chance of balls getting stuck
 *
 * this setting is to be decided as we finish the feeding system
 *
 */

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

DJIMotor jaspdexer(7, CANHandler::CANBUS_1, GIMBLY);
DigitalOut led(LED1);


int main(){

    unsigned long loopTimer = us_ticker_read() / 1000;
    bool previousMid;
    bool nowUp;
    bool nowDown;

    //remote.unfiltered = true;
    jaspdexer.justPosError = true;
    jaspdexer.setPositionOutputCap(100000);
    jaspdexer.setSpeedOutputCap(1000000);
    jaspdexer.setPositionIntegralCap(100000);
    jaspdexer.setSpeedIntegralCap(100000);
    jaspdexer.outCap = 100000;

    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    DJIMotor::sendValues();
    DJIMotor::getFeedback();

    int actualPosition = jaspdexer.getData(MULTITURNANGLE);
    float desiredPosition = float(int((actualPosition / dr)) * dr) + 200;
    int p;
    int t;
    float dp;

    printf("------------------------------------\n");
    printf("[i] %d\n", int(actualPosition));
    printf("[p] %d\n", int(desiredPosition));

    while (true) {
        unsigned long timeStart = us_ticker_read() / 1000;

        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            previousMid = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID);
            remoteRead();

            nowUp = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP);
            nowDown = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN);

            led = !led;
            actualPosition = jaspdexer.getData(MULTITURNANGLE);
            t = jaspdexer.getData(TORQUE);

            if(rS == Remote::SwitchState::DOWN) {
                if(previousMid && nowUp)
                    desiredPosition += dr;

                else if(previousMid && nowDown)
                    desiredPosition -= dr;

                dp = desiredPosition - float(actualPosition);
                p = 3000 * int(dp / dr + abs(dp) / dp);
                p += t;

                if(smooth){
                    p -= int((pow((dr - abs(dp)) / dr, 2.7) * t) * (abs(dp) / dp));
                }else{
                    p += int(1 * (-pow((abs(dp) - dr/2 - 30) / 8.35, 2) + 3000) * abs(dp) / dp);
                    p -= int((pow((dr - abs(dp)) / dr, 3) * t) * (abs(dp) / dp));
                }

                if (abs(p) > 32760)
                    p = (abs(p) / p) * 32760;

                if(abs(int(desiredPosition) - actualPosition) > 50)
                    jaspdexer.setPower(p);

                else
                    jaspdexer.setPower(0);

                printf("s %d\n", p);
                printf("torque: %d\n", t);

            }else if(rS == Remote::SwitchState::MID) {
                desiredPosition = float(int((actualPosition / dr)) * dr) + 200;
                jaspdexer.setPosition(int(desiredPosition));
                jaspdexer.setPower(0);

            }else
                jaspdexer.setPower(0);

            printf("pos: %d\n", int(desiredPosition));
            printf("POSITION: %d\n\n", actualPosition);

            DJIMotor::sendValues();
        }

        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
#pragma clang diagnostic pop