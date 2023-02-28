#include "main.h"
#include <cstdlib>

#define PI 3.14159265

#define LOWERBOUND 1000
#define UPPERBOUND 2000

// CANMotor LF(4,NewCANHandler::CANBUS_1,M3508);
// CANMotor RF(2,NewCANHandler::CANBUS_1,M3508);
// CANMotor LB(1,NewCANHandler::CANBUS_1,M3508);
// CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);


Chassis chassis(1, 2, 3, 4);
DigitalOut led(LED1);

//DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
//DJIMotor pitch(6, CANHandler::CANBUS_1, GIMBLY);
int pitchval = 0;

DJIMotor indexer(7, CANHandler::CANBUS_1, C610);
int indexJamTime = 0;
int lastJam = 0;

unsigned long cT = 0;
unsigned long forwardTime = 250;
unsigned long reverseTime = 300;
unsigned long totalTime;

bool sticksMoved = false;
int prevRS = 0, prevLS = 0;

PWMMotor RFLYWHEEL(D12); PWMMotor LFLYWHEEL(D11);
PWMMotor flyWheelMotors[] = {RFLYWHEEL, LFLYWHEEL};

void setFlyWheelPwr(int pwr) {
    for (int i = 0; i < 2; i++)
        flyWheelMotors[i].set(pwr);
}

Thread imuThread;

void runImuThread() {
    chassis.initializeImu();
    while (true) {
        chassis.readImu();
        ThisThread::sleep_for(25);
    }

}

int main()
{
    imuThread.start(runImuThread);
    float speedmultiplier = 3;
    float powmultiplier = 2;
    float translationalmultiplier = 1.5; // was 3
    float beybladespeedmult = 1;

    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);

    // LB.setSpeedPID(1.75, 0.351, 5.63);
    // RF.setSpeedPID(1.073, 0.556, 0);
    // RB.setSpeedPID(1.081, 0.247, 0.386);
    // LF.setSpeedPID(.743, 0.204, 0.284);
//    pitch.setPositionPID(4, 0.35, 0.35);
//    pitch.setPositionIntegralCap(10000);
//
//    pitch.useAbsEncoder = 1;
//    pitch.justPosError = 1;
//
//    yaw.setPositionPID(3.5, 0, 0.25);
//    yaw.setPositionIntegralCap(10000);
//    yaw.justPosError = 1;

    indexer.setSpeedPID(0.34, 0.002, 0.166);
    indexer.setSpeedIntegralCap(500000);

    chassis.setBrakeMode(Chassis::COAST);

    unsigned long loopTimer = us_ticker_read() / 1000;

    int indexJamTime = 0;

    bool strawberryJam = false;
    int refLoop=0;

    int yawSetpoint = 0;

    DJIMotor::getFeedback();
    double beybladeSpeed = 2;
    bool beybladeIncreasing = true;

    while (true) {
        led = !led;
        remoteRead();

        m3058_1

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;


            DJIMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
