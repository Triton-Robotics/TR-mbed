#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    // Initialize motor
    DJIMotor motor(3, CANHandler::CANBus::CANBUS_1, M3508, "MotorWeek4");
    motor.setPositionPID(1, 0, 0);
    motor.setSpeedPID(1,0,0);
    motor.pidPosition.setIntegralCap(2/0.01);

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            led = !led;

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
                printff("P: %fI: %fD: %f\n", 
                    motor.pidPosition.pComponent,
                    motor.pidPosition.iComponent,
                    motor.pidPosition.dComponent);
            }

            remoteRead();

            // Get left switch state
            Remote::SwitchState leftSwitchState = remote.leftSwitch();

            // Get left stick value
            int leftStickValue = remote.leftX();

            // Check switch mode (use scaling factor = 10)
            if ( leftSwitchState == Remote::SwitchState::UP ) {         // power mode
                motor.setPower(leftStickValue * 10);
                //printff("Mode: Power, Value: %d\n", leftStickValue * 10);
            }
            else if ( leftSwitchState == Remote::SwitchState::MID ) {   // speed mode
                motor.setSpeed(leftStickValue * 10);
                //printff("Mode: Speed, Value: %d\n", leftStickValue * 10);
            }
            else if ( leftSwitchState == Remote::SwitchState::DOWN ) {  // position mode
                motor.setPosition(leftStickValue * 5 + 3300);
                //printff("Mode: Position, Value: %d\n", leftStickValue * 10);
            }

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}