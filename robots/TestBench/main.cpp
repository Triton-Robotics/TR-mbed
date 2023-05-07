#include "main.h"
#include "mbed.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
DJIMotor jaspdexer(7, CANHandler::CANBUS_1, GIMBLY);
DigitalOut led(LED1);
//AnalogIn kp(A0);
//AnalogIn kI(A1);
//AnalogIn kD(A2);


// Check lS moves from 2 to 3 when the switch is moved up

int main(){

    unsigned long loopTimer = us_ticker_read() / 1000;
    float pos = 85;

    bool previousMid;
    bool nowUp;
    bool nowDown;

    //remote.unfiltered = true;
    jaspdexer.justPosError = true;
    jaspdexer.setPositionOutputCap(30000);
    jaspdexer.setSpeedOutputCap(50000);

    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    DJIMotor::getFeedback();

    while (true) {
        unsigned long timeStart = us_ticker_read() / 1000;

        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            previousMid = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID);
            remoteRead();

            nowUp = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP);
            nowDown = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN);

            led = !led;

            if(rS == Remote::SwitchState::MID) {
                if(previousMid && nowUp)
                    pos += 8191.0 / 9;

                else if(previousMid && nowDown)
                    pos -= 8191.0 / 9;

                jaspdexer.setPosition(int(pos));
                printf("POS: %d\n", int(pos));

            }else if(rS == Remote::SwitchState::DOWN) {
                pos = 85;
                jaspdexer.setPosition(int(pos));
                printf("POS: %d\n", int(pos));

            }else
                jaspdexer.setPower(0);

            printf("POSITION: %d\n\n", jaspdexer.getData(MULTITURNANGLE));

            DJIMotor::sendValues();
        }

        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
#pragma clang diagnostic pop