// Yaw or Pitch response reading

#include "main.h"
#include <cmath>

// Testing Parameters
bool infantry = true, sentry = false, hero = false;
bool pitch = false, yaw = true;
bool ind = false;
bool chassis = false;
bool position = false, velocity = true;

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

DJIMotor sentryPitch(5, CANHandler::CANBUS_2, GM6020, "sentry_pitch_motor");
DJIMotor sentryYaw1(6, CANHandler::CANBUS_1, GM6020, "sentry_yaw_motor_1");
DJIMotor sentryYaw2(7, CANHandler::CANBUS_1, GM6020, "sentry_yaw_motor_2");
DJIMotor infPitch(7, CANHandler::CANBUS_2, GM6020, "infantry_pitch_motor");
DJIMotor infYaw(4, CANHandler::CANBUS_1, GM6020, "infantry_yaw_motor");
DJIMotor indexer(2, CANHandler::CANBUS_1, M3508, "indexer");
DJIMotor heroPitch(5, CANHandler::CANBUS_2, GM6020, "hero_pitch_motor");
DJIMotor heroYaw(1, CANHandler::CANBUS_1, GM6020, "hero_yaw_motor");

// DJIMotor motor1(1, CANHandler::CANBUS_1, M3508, "chassis motor 1");
// DJIMotor motor2(2, CANHandler::CANBUS_1, M3508, "chassis motor 2");
// DJIMotor motor3(3, CANHandler::CANBUS_1, M3508, "chassis motor 3");
// DJIMotor motor4(4, CANHandler::CANBUS_1, M3508, "chassis motor 4");

#define IMPULSE_STRENGTH 4095
#define REMOTE_MAX 660

int main(){
    usbSerial.set_blocking(false);

    DJIMotor *testMot = nullptr;
    DJIMotor *testMot2 = nullptr;
    DJIMotor *testMot3 = nullptr;
    DJIMotor *testMot4 = nullptr;

    if (infantry && pitch) {
        testMot = &infPitch;
    }
    // else if (chassis) {
        // testMot = &motor1;
        // testMot2 = &motor2;
        // testMot3 = &motor3;
        // testMot4 = &motor4;
    // }
    else if (infantry && yaw)
         testMot = &infYaw;
    else if (sentry && pitch)
         testMot = &sentryPitch;
    else if (sentry && yaw) {
        testMot = &sentryYaw1;
        testMot2 = &sentryYaw2;
    }
    else if (hero && pitch)
         testMot = &heroPitch;
    else if (hero && yaw)
         testMot = &heroYaw;
    else if (ind)
        testMot = &indexer;

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    bool prevL = false;
    bool switL = false;
    bool debug = false;

    int motorSpeed = 0;
    int powerValue = 0;
    int ry = 0, desiredVelocity = 0;
    int stepAmplitude = IMPULSE_STRENGTH;
    int16_t powerBuffer = 0;
    int16_t velocityBuffer = 0, angleBuffer = 0, torqueBuffer = 0;
    bool impulse = true, measureStart = false;

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 2){
           loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;

            refLoop++;
            if (refLoop >= 10){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
            }
            prevL = switL;
            remoteRead();

            switL = (remote.leftSwitch() == Remote::SwitchState::UP);
            bool switLDown = (remote.leftSwitch() == Remote::SwitchState::DOWN);
            bool switR = (remote.rightSwitch() == Remote::SwitchState::UP);
            bool switRDown = (remote.rightSwitch() == Remote::SwitchState::DOWN);

            if (switL) { // step response
                powerBuffer = stepAmplitude;
            }
            else if (switLDown) { // inverted step
                powerBuffer = -stepAmplitude;
            }
            else if (switR) {  // arbitrary response
                ry = remote.rightY();
//                powerBuffer = -IMPULSE_STRENGTH + ((float) ry+REMOTE_MAX)/(REMOTE_MAX*2) * (2*IMPULSE_STRENGTH);
                powerBuffer = IMPULSE_STRENGTH*(2/(1 + exp(.002*ry)) - 1);
            }
            else if (switRDown) { // constant power increase
                if (!measureStart) {
                    measureStart = true;
                    powerBuffer = 0;
                }
                else if (powerBuffer > IMPULSE_STRENGTH) {
                    powerBuffer = 0;
                }
                else
                    powerBuffer++;
            }
            else {
                measureStart = false;
                powerBuffer = 0;
            }

            testMot->setPower(powerBuffer);
            testMot2 ? testMot2->setPower(powerBuffer) : (void)0; // for double motor on sentry yaw
            testMot3 ? testMot3->setPower(powerBuffer) : (void)0; // for chassis motor
            testMot4 ? testMot4->setPower(powerBuffer) : (void)0; // for chassis motor
            angleBuffer = (*testMot)>>ANGLE;
            velocityBuffer = (*testMot)>>VELOCITY;
            torqueBuffer = (*testMot)>>TORQUE;

            if (switL || switR || switLDown || switRDown)  // print only when test is active
                if (position)
                    printff("%d\t%d\n", powerBuffer, angleBuffer);
                else if (velocity)
                    printff("%d\t%d\n", powerBuffer, velocityBuffer);
            DJIMotor::s_sendValues();
        }
        printff("%d\n", testMot->getData(ANGLE));
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}