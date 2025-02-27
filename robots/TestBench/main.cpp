// #include "main.h"

// DigitalOut led(L27);
// DigitalOut led2(L26);
// DigitalOut led3(L25);
// DigitalOut ledbuiltin(LED1);

// DJIMotor testMot(5, CANHandler::CANBUS_1, GM6020, "testbench_motor");

// #define IMPULSE_DT 100
// #define IMPULSE_STRENGTH 16383

// int main(){

//     DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
//     DJIMotor::s_sendValues();
//     DJIMotor::s_getFeedback();

//     unsigned long timeStart;
//     unsigned long loopTimer = us_ticker_read();
//     int refLoop = 0;

//     testMot.setSpeedPID(1.5,0,200);

//     bool prevL = false;
//     bool switL = false;

//     int motorSpeed = 0;

//     while(true){
//         timeStart = us_ticker_read();

//         if ((timeStart - loopTimer) / 1000 > 25){
//             loopTimer = timeStart;
//             led = !led;
//             ledbuiltin = !ledbuiltin;

//             refLoop++;
//             if (refLoop >= 5){
//                 refereeThread(&referee);
//                 refLoop = 0;
//                 led2 = !led2;
//             }
//             prevL = switL;
//             remoteRead();
//             switL = (remote.leftSwitch() == Remote::SwitchState::UP);

//             // testMot.setPower(remote.leftX() * 3);
//             // if(!prevL && switL){
//             //     motorSpeed += 10;
//             // }

//             motorSpeed = remote.leftX() / 6;

//             testMot.setSpeed(motorSpeed);
//             int dir = 0;
//             if(motorSpeed > 0){
//                 dir = 1;
//             }else if(motorSpeed < 0){
//                 dir = -1;
//             }
//             testMot.pidSpeed.feedForward = dir * (874 + (73.7 * abs(motorSpeed)) + (0.0948 * motorSpeed * motorSpeed));
//             printff("%d\t%d\t%d\n", testMot>>POWEROUT, motorSpeed, testMot>>VELOCITY);

//             DJIMotor::s_sendValues();
//         }
//         DJIMotor::s_getFeedback();
//         ThisThread::sleep_for(1ms);
//     }
// }

//STARTER 
//THIS EXERCISE IS TO BE DONE AND THEN RUN IN REAL LIFE ON A ROBOT
//YOU WILL NOT BE ABLE TO COMPILE THIS UNLESS YOU SET UP THE BUILD ENVIRONMENT
#include "main.h"

DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

I2C i2c(I2C_SDA, I2C_SCL);

//DEFINE MOTORS, ETC
const int RPM_MAX = 9000;
const int REMOTE_MAX = 660;
const int RPM_REMOTE_RATIO = RPM_MAX / REMOTE_MAX;

DJIMotor* frontleft = new DJIMotor(1, CANHandler::CANBus::CANBUS_2, motorType::M3508, "FL");

DJIMotor* frontright = new DJIMotor(2, CANHandler::CANBus::CANBUS_2, motorType::M3508, "FR");

DJIMotor* backleft = new DJIMotor(3, CANHandler::CANBus::CANBUS_2, motorType::M3508, "BL");

DJIMotor* backright = new DJIMotor(4, CANHandler::CANBus::CANBUS_2, motorType::M3508, "BR");


int main(){

    //assigning can handler objects to motor class.
    DJIMotor::s_setCANHandlers(&canHandler1,&canHandler2, false, false); 

    //getting initial feedback.
    DJIMotor::s_getFeedback();

    unsigned long loopTimer_u = us_ticker_read();
    unsigned long timeEnd_u;
    unsigned long timeStart_u;

    int refLoop = 0;

    //DEFINE PIDs AND OTHER CONSTANTS

    while(true){ //main loop
        timeStart_u = us_ticker_read();

        //inner loop runs every 25ms
        if((timeStart_u - loopTimer_u) / 1000 > 25) { 
            loopTimer_u = timeStart_u;
            led = !led; //led blink tells us how fast the inner loop is running

            if (refLoop >= 5) { //ref code runs 5 of every inner loop, 
                refLoop = 0;
                refereeThread(&referee);

                printff("%d %d %d %d\n", *frontleft>>ANGLE, *frontright>>ANGLE, *backleft>>ANGLE, *backright>>ANGLE);
            }
            refLoop ++;

            remoteRead(); //reading data from remote
        
            //MAIN CODE
            int flspeed = 0;
            int frspeed = 0; 
            int blspeed = 0;
            int brspeed = 0;

            int lx = 0;
            int ly = 0;
            int rx = 0;

            if (abs(remote.leftY()) > 20) {
                ly = remote.leftY();
            }
            if (abs(remote.leftX()) > 20) {
                lx = remote.leftX();
            }
            if (abs(remote.rightX()) > 20) {
                rx = remote.rightX();
            }

            flspeed += ((ly + lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;

            frspeed += ((-ly + lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;

            blspeed += ((-ly - lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;

            brspeed += (( ly - lx + rx) * RPM_REMOTE_RATIO) % RPM_MAX;

            frontleft->setSpeed(flspeed);
            frontright->setSpeed(frspeed);
            backleft->setSpeed(blspeed);
            backright->setSpeed(brspeed);

            //MOST CODE DOESNT NEED TO RUN FASTER THAN EVERY 25ms

            timeEnd_u = us_ticker_read();

            DJIMotor::s_sendValues();
        }

        //FEEDBACK CODE DOES NEED TO RUN FASTER THAN 1MS
        //OTHER QUICK AND URGENT TASKS GO HERE

        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}