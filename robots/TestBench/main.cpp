// Yaw or Pitch response reading

#include "main.h"
#include <cmath>

// Testing Parameters
bool infantry = true, sentry = false, hero = false;
bool pitch = false, yaw = true;
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
DJIMotor heroPitch(5, CANHandler::CANBUS_2, GM6020, "hero_pitch_motor");
DJIMotor heroYaw(1, CANHandler::CANBUS_1, GM6020, "hero_yaw_motor");

#define IMPULSE_STRENGTH 8191
#define REMOTE_MAX 660

int main(){

    DJIMotor *testMot = nullptr;
    DJIMotor *testMot2 = nullptr;

    if (infantry || pitch) {
        testMot = &infPitch;
    }
    else if (infantry || yaw)
         testMot = &infYaw;
    else if (sentry || pitch)
         testMot = &sentryPitch;
    else if (sentry || yaw) {
        testMot = &sentryYaw1;
        testMot2 = &sentryYaw2;
    }
    else if (hero || pitch)
         testMot = &heroPitch;
    else if (hero || yaw)
         testMot = &heroYaw;

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

        if ((timeStart - loopTimer) / 1000 > 15){
//            loopTimer = timeStart;
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
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}



//// Test drive motors
//#include "main.h"
//
//DigitalOut led(L26);
//DigitalOut led2(L27);
//DigitalOut led3(L25);
//
//I2C i2c(I2C_SDA, I2C_SCL);
//
//  //DEFINE MOTORS, ETC
//const int RPM_MAX = 9000;
//const int REMOTE_MAX = 660;
//const int RPM_REMOTE_RATIO = RPM_MAX / REMOTE_MAX;
//
//DJIMotor* frontleft = new DJIMotor(1, CANHandler::CANBus::CANBUS_1, motorType::M3508, "FL");
//
//DJIMotor* frontright = new DJIMotor(2, CANHandler::CANBus::CANBUS_1, motorType::M3508, "FR");
//
//DJIMotor* backleft = new DJIMotor(3, CANHandler::CANBus::CANBUS_1, motorType::M3508, "BL");
//
//DJIMotor* backright = new DJIMotor(4, CANHandler::CANBus::CANBUS_1, motorType::M3508, "BR");
//
//
//int main(){
//
//    //assigning can handler objects to motor class.
//    DJIMotor::s_setCANHandlers(&canHandler1,&canHandler2, false, false);
//
//    //getting initial feedback.
//    DJIMotor::s_getFeedback();
//
//    unsigned long loopTimer_u = us_ticker_read();
//    unsigned long timeEnd_u;
//    unsigned long timeStart_u;
//
//    frontleft->setSpeedPID(5.2908, .010269, 0);
//    frontright->setSpeedPID(5.2908, .010269, 0);
//    backleft->setSpeedPID(5.2908, .010269, 0);
//    backright->setSpeedPID(5.2908, .010269, 0);
//
//    int refLoop = 0;
//
//    //DEFINE PIDs AND OTHER CONSTANTS
//
//    int flspeed = 0;
//    int frspeed = 0;
//    int blspeed = 0;
//    int brspeed = 0;
//
//    while(true){ //main loop
//        timeStart_u = us_ticker_read();
//
//        //inner loop runs every 25ms
//        if((timeStart_u - loopTimer_u) / 1000 > 25) {
//            loopTimer_u = timeStart_u;
//            led = !led; //led blink tells us how fast the inner loop is running
//
//            if (refLoop >= 5) { //ref code runs 5 of every inner loop,
//                refLoop = 0;
//                refereeThread(&referee);
//
//                printff("%d %d %d %d", *frontleft>>VELOCITY, *frontright>>VELOCITY, *backleft>>VELOCITY, *backright>>VELOCITY);
//                printff(" %d %d %d %d\n", flspeed, frspeed, blspeed, brspeed);
//            }
//            refLoop ++;
//
//            remoteRead(); //reading data from remote
//
//            //MAIN CODE
//            flspeed = 0;
//            frspeed = 0;
//            blspeed = 0;
//            brspeed = 0;
//
//            int lx = 0;
//            int ly = 0;
//            int rx = 0;
//
//            if (abs(remote.leftY()) > 20) {
//                ly = remote.leftY();
//            }
//            if (abs(remote.leftX()) > 20) {
//                lx = remote.leftX();
//            }
//            if (abs(remote.rightX()) > 20) {
//                rx = remote.rightX();
//            }
//
//            flspeed = ((ly + lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;
//
//            frspeed = ((-ly + lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;
//
//            blspeed = ((ly - lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;
//
//            brspeed = ((-ly - lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;
//
//            frontleft->setSpeed(flspeed);
//            frontright->setSpeed(frspeed);
//            backleft->setSpeed(blspeed);
//            backright->setSpeed(brspeed);
//
//            //MOST CODE DOESNT NEED TO RUN FASTER THAN EVERY 25ms
//
//            timeEnd_u = us_ticker_read();
//
//            DJIMotor::s_sendValues();
//        }
//
//        //FEEDBACK CODE DOES NEED TO RUN FASTER THAN 1MS
//        //OTHER QUICK AND URGENT TASKS GO HERE
//
//        DJIMotor::s_getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}