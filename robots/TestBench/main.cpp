#include "main.h"
#include "mbed.h"

#define dr (8191.0 / 9.0)

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

                p = 3000 * int(((desiredPosition - float(actualPosition)) / dr) + abs(desiredPosition - float(actualPosition)) / (desiredPosition - float(actualPosition)));
                p += t;
                p -= int((pow((dr - abs(desiredPosition - float(actualPosition))) / dr, 2.7) * t) * abs(desiredPosition - float(actualPosition)) / (desiredPosition - float(actualPosition)));

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