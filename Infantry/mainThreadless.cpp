#include "../src/main.hpp"
#include <cstdlib>
#include "../src/subsystems/Chassis.cpp"

#define PI 3.14159265

// CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); 
// CANMotor RF(2,NewCANHandler::CANBUS_1,M3508); 
// CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); 
// CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);

Chassis chassis(1, 2, 3, 4);
DigitalOut led(LED1);

CANMotor yaw(5, NewCANHandler::CANBUS_1, GIMBLY);
CANMotor pitch(6, NewCANHandler::CANBUS_1, GIMBLY);
int pitchval = 0;
#define LOWERBOUND 1000
#define UPPERBOUND 2000

CANMotor indexer(7, NewCANHandler::CANBUS_1, C610);
int indexJamTime = 0;
int lastJam = 0;

unsigned long cT = 0;
unsigned long forwardTime = 250;
unsigned long reverseTime = 300;
unsigned long totalTime;

PWMMotor RFLYWHEEL(D12); PWMMotor LFLYWHEEL(D11);
PWMMotor flyWheelMotors[] = {RFLYWHEEL, LFLYWHEEL};

void setFlyWheelPwr(int pwr) {
    for (int i = 0; i < 2; i++)
        flyWheelMotors[i].set(pwr);
}

int main()
{
    float speedmultiplier = 3;
    float powmultiplier = 2;
    float translationalmultiplier = 3;
    float beybladespeedmult = 1;

    CANMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);

    // LB.setSpeedPID(1.75, 0.351, 5.63);
    // RF.setSpeedPID(1.073, 0.556, 0);
    // RB.setSpeedPID(1.081, 0.247, 0.386);
    // LF.setSpeedPID(.743, 0.204, 0.284);
    pitch.setPositionPID(4, 0.35, 0.35);
    pitch.setPositionIntegralCap(10000);


    // pitch.setPositionPID(.017,.001,.044);
    pitch.useAbsEncoder = 1;
    pitch.justPosError = 1;
    yaw.setSpeedPID(78.181, 7.303, 1.227);
    indexer.setSpeedPID(0.34, 0.002, 0.166);
    indexer.setSpeedIntegralCap(500000);

    chassis.setBrakeMode(COAST);

    // LF.outCap = 16000;   
    // RF.outCap = 16000;
    // LB.outCap = 16000;
    // RB.outCap = 16000;

    unsigned long loopTimer = us_ticker_read() / 1000;

    int indexJamTime = 0;
    int lastJam = 0;

    bool strawberryJam = false;

    while (true) {
        led = !led;
        remoteRead();

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 10){
            loopTimer = timeStart;

            if(rS == 1){ // All non-serializer motors activated
                int LFa = lY + lX*translationalmultiplier + rX, RFa = lY - lX*translationalmultiplier - rX, LBa = lY - lX*translationalmultiplier + rX, RBa = lY + lX*translationalmultiplier - rX;
                chassis.driveXYR(lX / 500.0, lY / 500.0, rX / 500.0);
                // LF.setSpeed(LFa * speedmultiplier);
                // RF.setSpeed(-RFa * speedmultiplier);
                // LB.setSpeed(LBa * speedmultiplier);
                // RB.setSpeed(-RBa * speedmultiplier);

                // LF.setPower(LFa * powmultiplier);
                // RF.setPower(-RFa * powmultiplier);
                // LB.setPower(LBa * powmultiplier);
                // RB.setPower(-RBa * powmultiplier);
                
                // pitch.setPower(rY*9);
                pitch.setPosition((rY / 2) + 1500);
                //yaw.setSpeed(rX/100);
                

            }else if(rS == 2){ //disable all the non-serializer components
                // LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
                yaw.setPower(0); pitch.setPower(0);
            }else if(rS == 3){ // beyblade mode
                // LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
                yaw.setPower(0); pitch.setPower(0);
            }

            if (lS == 3) {
                //indexer.setPower(1200);
                indexer.setSpeed(-4500);
                setFlyWheelPwr(40);
                
            }else if(lS == 2){ //disable serializer
                indexer.setPower(0);
                setFlyWheelPwr(0);
            }else if(lS == 1){
                ///////////////////////////////////////////
                /// THEO SECTION OF CODE
                ///////////////////////////////////////////
                printf("TORQ:%d VEL:%d\n",indexer.getData(TORQUE), indexer.getData(VELOCITY));
                if(abs(indexer.getData(TORQUE)) > 100 & abs(indexer.getData(VELOCITY)) < 20){ //jam
                    indexJamTime = us_ticker_read() /1000;
                }
                if(us_ticker_read() / 1000 - indexJamTime < 1000){
                    indexer.setPower(14000); //jam
                    printf("JAMMMMM- ");
                }else if(us_ticker_read() / 1000 - indexJamTime < 1500){
                    indexer.setPower(-9000); //jam
                    printf("POWER FORWARD- ");
                }else{
                    //indexer.setPower(-900);   
                    indexer.setSpeed(-4500);
                }
                LFLYWHEEL.set(40); RFLYWHEEL.set(40);
            }
            CANMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        CANMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
