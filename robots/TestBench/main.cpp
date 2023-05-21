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
    int s = 0;
    float desiredPosition = float(int((actualPosition / (8191.0 / 9.0))) * (8191.0 / 9.0)) + 100;

    printf("------------------------------------\n");
    printf("[i] %d\n", int(actualPosition));
    printf("[p] %d\n", int(desiredPosition));

    //ThisThread::sleep_for(5000ms);


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

            if(rS == Remote::SwitchState::UP) {
                if(previousMid && nowUp)
                    desiredPosition += 8191.0 / 9;

                else if(previousMid && nowDown)
                    desiredPosition -= 8191.0 / 9;

                jaspdexer.setPosition(int(desiredPosition));

            }else if(rS == Remote::SwitchState::DOWN) {
                if(previousMid && nowUp)
                    desiredPosition += 8191.0 / 9;

                else if(previousMid && nowDown)
                    desiredPosition -= 8191.0 / 9;

                s = 3000 * (int((desiredPosition - actualPosition) / (8191.0 / 9.0)) + 1);
                if (s > 32760)
                    s = 32760;

                if(int(desiredPosition) > actualPosition)
                    jaspdexer.setPower(s);

                else
                    jaspdexer.setPower(0);

                printf("s %d\n", s);

            }else if(rS == Remote::SwitchState::MID) {
                desiredPosition = float(int((actualPosition / (8191.0 / 9.0))) * (8191.0 / 9.0)) + 100;
                jaspdexer.setPosition(int(desiredPosition));
                jaspdexer.setPower(0);
            }

            printf("pos: %d\n", int(desiredPosition));
            printf("POSITION: %d\n\n", actualPosition);

            DJIMotor::sendValues();
        }

        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
#pragma clang diagnostic pop