//// Response reading
//
//#include "main.h"
//#include <cmath>
//
//// Testing Parameters
//bool speed = true;
//bool position = false;
//
//DigitalOut led(L27);
//DigitalOut led2(L26);
//DigitalOut led3(L25);
//DigitalOut ledbuiltin(LED1);
//
//DJIMotor testMot(6, CANHandler::CANBUS_1, GM6020, "testbench_motor");  // sentry yaw 1
//DJIMotor testMot2(7, CANHandler::CANBUS_1, GM6020, "testbench_motor");  // sentry yaw 1
////DJIMotor testMot(5, CANHandler::CANBUS_2, GM6020, "testbench_motor"); // sentry pitch
////DJIMotor testMot(7, CANHandler::CANBUS_2, GM6020, "testbench_motor"); // inf pitch
//
//#define IMPULSE_STRENGTH 16383
//#define REMOTE_MAX 660
//
//int main(){
//
//    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
//    DJIMotor::s_sendValues();
//    DJIMotor::s_getFeedback();
//
//    unsigned long timeStart;
//    unsigned long loopTimer = us_ticker_read();
//    int refLoop = 0;
//
//    bool prevL = false;
//    bool switL = false;
//    bool debug = false;
//
//    int motorSpeed = 0;
//    int powerValue = 0;
//    int ry = 0, desiredVelocity = 0;
//    int stepAmplitude = IMPULSE_STRENGTH * 0.5;
//    int16_t powerBuffer = 0;
//    int16_t velocityBuffer = 0, angleBuffer = 0, torqueBuffer = 0;
//    bool impulse = true, measureStart = false;
//
//    while(true){
//        timeStart = us_ticker_read();
//
//        if ((timeStart - loopTimer) / 1000 > 15){
////            loopTimer = timeStart;
//            led = !led;
//            ledbuiltin = !ledbuiltin;
//
//            refLoop++;
//            if (refLoop >= 5){
//                refereeThread(&referee);
//                refLoop = 0;
//                led2 = !led2;
//            }
//            prevL = switL;
//            remoteRead();
//
//            switL = (remote.leftSwitch() == Remote::SwitchState::UP);
//            bool switLDown = (remote.leftSwitch() == Remote::SwitchState::DOWN);
//            bool switR = (remote.rightSwitch() == Remote::SwitchState::UP);
//            bool switRDown = (remote.rightSwitch() == Remote::SwitchState::DOWN);
//
//            if (switL) { // step response
//                powerBuffer = stepAmplitude;
//            }
//            else if (switLDown) { // inverted step
//                powerBuffer = -stepAmplitude;
//            }
//            else if (switR) {  // arbitrary response
//                ry = remote.rightY();
////                powerBuffer = -IMPULSE_STRENGTH + ((float) ry+REMOTE_MAX)/(REMOTE_MAX*2) * (2*IMPULSE_STRENGTH);
//                powerBuffer = IMPULSE_STRENGTH*(2/(1 + exp(.002*ry)) - 1);
//            }
//            else if (switRDown) { // constant power increase
//                if (!measureStart) {
//                    measureStart = true;
//                    powerBuffer = 0;
//                }
//                else if (powerBuffer > IMPULSE_STRENGTH) {
//                    powerBuffer = 0;
//                }
//                else
//                    powerBuffer++;
//            }
//            else {
//                measureStart = false;
//                powerBuffer = 0;
//            }
//
//            testMot.setPower(powerBuffer);
//            testMot2.setPower(powerBuffer); // for 2 motor yaw
//            angleBuffer = testMot>>ANGLE;
//            velocityBuffer = testMot>>VELOCITY;
//            torqueBuffer = testMot>>TORQUE;
//
//            if (switL || switR || switLDown || switRDown)  // print only when test is active
//                if (position)
//                    printff("%d\t%d\n", powerBuffer, angleBuffer);
//                else if (speed)
//                    printff("%d\t%d\n", powerBuffer, velocityBuffer);
//            DJIMotor::s_sendValues();
//        }
//        DJIMotor::s_getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}



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

// Test yaw or pitch motor
#include "main.h"
#include <cmath>

// Testing Parameters
bool pitch = false;
bool yaw = false;

DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

I2C i2c(I2C_SDA, I2C_SCL);

//DEFINE MOTORS, ETC
const int RPM_MAX = 9000;
const int REMOTE_MAX = 660;
const int RPM_REMOTE_RATIO = RPM_MAX / REMOTE_MAX;

// Change based on turret constraints
#define PITCH_MIN 2400
#define PITCH_MAX 300
#define YAW_MIN 1
#define YAW_MAX 8192
#define MAX_POWER 16383

//    DJIMotor testMot(5, CANHandler::CANBUS_2, GM6020, "testbench_motor"); // sentry pitch
    DJIMotor testMot(6, CANHandler::CANBUS_1, GM6020, "testbench_motor");  // sentry yaw 1
    DJIMotor testMot2(7, CANHandler::CANBUS_1, GM6020, "testbench_motor"); // sentry yaw 2
//    DJIMotor testMot(7, CANHandler::CANBUS_2, GM6020, "testbench_motor");  // infantry pitch
    //DJIMotor testMot(4, CANHandler::CANBUS_1, GM6020, "testbench_motor");  // infantry yaw
    //DJIMotor testMot(5, CANHandler::CANBUS_2, GM6020, "testbench_motor");  // hero pitch
//    DJIMotor testMot(1, CANHandler::CANBUS_1, GM6020, "testbench_motor");  // hero yaw

int pitchPos = PITCH_MIN, yawPos = YAW_MIN;
int ry = 0, yawPower = 0;

int main(){
    int maxPower = MAX_POWER;
//    testMot.outputCap = maxPower + 1000;
//    testMot2.outputCap = maxPower + 1000;

    //assigning can handler objects to motor class.
    DJIMotor::s_setCANHandlers(&canHandler1,&canHandler2, false, false);

    //getting initial feedback.
    DJIMotor::s_getFeedback();

    unsigned long loopTimer_u = us_ticker_read();
    unsigned long timeEnd_u;
    unsigned long timeStart_u;

    /* GOOD PID VALUES
     * inf pitch pos pid: (16.2644,0.024926,1200.2213), ff = -3550, icap = 3000
     * sentry yaw spd pid: (937.48, 0.48305, 0),
     *  ff = (-5.30094881524873 * pow(desiredSpeed,2)*dir + 461.129143101395 * desiredSpeed + 2402.35249010233*dir),
     *  icap = 5000
     *
     *
     */


    // infantry pitch
//     testMot.setPositionPID(17.4351,0.024875,1053.175); // old
//    testMot.setPositionPID(16.2644,0.024926,445.2213);  // new
//    testMot.setPositionPID(16.2644,0.024926,1200.2213);  // good w/ ff
//    testMot.setPositionIntegralCap(3000);

    // infantry yaw
    // testMot.setPositionPID(3.0459,0.00058304,3977.9767);
    // testMot.setPositionIntegralCap();

//    // sentry pitch
//    testMot.setPositionPID(21.3544, 0.020221, 1078.4383);
//    testMot.setPositionPID(0.51595, 3.6503e-05, 1823.1593);
//    testMot.setPositionIntegralCap(3000);

    // sentry yaw (ff: 2.146e-07 x^2 + 0.000247 x - 1.501)
//    testMot.setSpeedPID(1883.2412, 2.5103, 0); // old
    testMot.setSpeedPID(922.9095, 0.51424, 0); // new
//    testMot2.setSpeedPID(288.6194, 0.15308, 0);
//    testMot.setPositionPID(23.8829, 0.0012624, 0);
//    yawMot.setPositionIntegralCap(500);
    testMot.setSpeedIntegralCap(1000);
    testMot2.setSpeedIntegralCap(1000);

    // hero pitch
//    testMot.setPositionPID(18.22, 0.012889, 439.1312);
//    testMot.setPositionPID(2.7811, 0.0031165, 620.4491);

    // hero yaw
//    testMot.setPositionPID(10.4569, 0.0012426, 7681.1044);
//    testMot.setPositionPID(6.369, 0.00084055, 9283.1751);
    int refLoop = 0;

    //DEFINE PIDs AND OTHER CONSTANTS

    int desiredSpeed = 0;

    while(true){ //main loop
        timeStart_u = us_ticker_read();

        //inner loop runs every 25ms
        if((timeStart_u - loopTimer_u) / 1000 > 15) {
            loopTimer_u = timeStart_u;
            led = !led; //led blink tells us how fast the inner loop is running

            remoteRead(); //reading data from remote
            bool switL = (remote.leftSwitch() == Remote::SwitchState::UP);

            if (refLoop >= 5) { //ref code runs 10 of every inner loop,
                refLoop = 0;
                refereeThread(&referee);

                int curAngle = testMot>>ANGLE;
                int pitchError = abs(pitchPos - curAngle);
                int yawError = abs(yawPos - curAngle);
                int speedError = abs(desiredSpeed - (testMot>>VELOCITY));
//                int iCPos = testMot.pidPosition.iC;
                int iCSpd = testMot.pidSpeed.iC;
//                printff("%d\t%d\t%d\n", pitchError, testMot>>ANGLE, iCPos);
                if (switL)
                    printff("%d\t%d\t%d\t%d\t%d\t%d\n", speedError, desiredSpeed , testMot>>VELOCITY, iCSpd, testMot.powerOut, testMot2.powerOut);
//                printff("%d\t%d\t%d\n", yawError, testMot>>ANGLE, yawPos);
            }
            refLoop ++;

            int lx = 0;
            int ly = 0;
            int rx = 0;
//            int ry = 0;
            ry = 0;

            if (abs(remote.rightX()) > 35) {
                rx = remote.rightX();
            }
            if (abs(remote.rightY()) > 35) {
                ry = remote.rightY();
            }
            if (abs(remote.leftX()) > 35) {
                lx = remote.leftX();
            }
            if (abs(remote.leftY()) > 35) {
                ly = remote.leftY();
            }

//            pitchPos = PITCH_MIN + ((float) ry+REMOTE_MAX)/(REMOTE_MAX*2) * (PITCH_MAX - PITCH_MIN);
//            yawPos = YAW_MIN + ((float) rx+REMOTE_MAX)/(REMOTE_MAX*2) * (YAW_MAX - YAW_MIN);
            desiredSpeed = ry / 10;
//            yawPower = 60 * (float) rx/(REMOTE_MAX);
//            if (testMot>>ANGLE > PITCH_MAX*.95) {
//                pitchPos = PITCH_MIN;
//            }
//            else {
//                pitchPos += (PITCH_MAX - PITCH_MIN) / 400;
//            }

            float pitchDelta = pitchPos - (testMot>>ANGLE);
//            testMot.pidPosition.feedForward = -3550; // infantry pitch pos ff
//            int x = desiredSpeed;
            int dir = 0;
            if(desiredSpeed > 0){
                dir = 1;
            }else if(desiredSpeed < 0){
                dir = -1;
            }
            // sentry yaw speed ff
//            testMot.pidSpeed.feedForward = (-5.30094881524873 * pow(desiredSpeed,2)*dir
//                                            + 461.129143101395 * abs(desiredSpeed) + 2402.35249010233*dir);
            testMot.pidSpeed.feedForward = (-0.431808886065578*pow(desiredSpeed,2))*dir
                                            + 167.091229447692*desiredSpeed + 1643.17393278750*dir;
            if (switL) {
//                testMot.setPosition(yawPos);
//                testMot.setPosition(pitchPos);
                testMot.setSpeed(desiredSpeed);
                int powerOut = testMot.powerOut;
                int powerDir = 0;
                if (powerOut > 0)
                    powerDir = 1;
                else if (powerOut < 0)
                    powerDir = -1;
                if (abs(powerOut) >= maxPower) {
                    testMot2.setPower(powerOut - maxPower*powerDir);
//                    printff("on\n");
                }
//                testMot2.setPower(ry * 10);
//                testMot.setPower(ry * 10);
            }
            else {
                testMot.setPower(0);
                testMot2.setPower(0);
//            yawMot.setSpeed(yawPower);
//            testMot.setPower(0);
            }

            timeEnd_u = us_ticker_read();

            DJIMotor::s_sendValues();
        }

        //FEEDBACK CODE DOES NEED TO RUN FASTER THAN 1MS
        //OTHER QUICK AND URGENT TASKS GO HERE

        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
