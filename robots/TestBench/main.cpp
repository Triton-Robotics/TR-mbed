

#include "main.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define PI 3.14159265

#define dr (8191.0 / 9.0)
#define IOFFSET 200
#define smooth true

#define LOWERBOUND 1000
#define UPPERBOUND 2000

DigitalOut led(LED1);
I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);

DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor indexer(7, CANHandler::CANBUS_1, GIMBLY);

//DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2,M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2,M3508);

Remote::SwitchState prevRS = Remote::SwitchState::UNKNOWN, prevLS = Remote::SwitchState::UNKNOWN;

Thread imuThread;
bool sticksMoved = false;

void setFlyWheelPwr(int pwr){
     RFLYWHEEL.setPower(pwr);
     LFLYWHEEL.setPower(-pwr);
}

void runImuThread(){
    chassis.initializeImu();
    while (true) {
        chassis.readImu();
        ThisThread::sleep_for(25);
    }
}

void setMotorSettings(){

//    pitch.setPositionPID(4, 0.85, 0.15);
//    pitch.pidPosition.setIntegralCap(3000);
//    pitch.setPositionIntegralCap(10000);
//    pitch.useAbsEncoder = true;
//    pitch.justPosError = true;

    yaw.setPositionPID(3.5, 0, 0.25);
    yaw.setPositionIntegralCap(10000);
    yaw.justPosError = true;

    indexer.justPosError = true;
    indexer.setPositionOutputCap(100000);
    indexer.setSpeedOutputCap(1000000);
    indexer.setPositionIntegralCap(100000);
    indexer.setSpeedIntegralCap(100000);
    indexer.outCap = 100000;

    LFLYWHEEL.outCap = 16384;
    RFLYWHEEL.outCap = 16384;

    chassis.setBrakeMode(Chassis::COAST);

}

void indexerLoop(bool &previousMid, bool &nowUp, bool &nowDown, int &actualPosition, float &desiredPosition, int &t){

    float dp;
    int p;

    if(previousMid && nowUp)
        desiredPosition += dr;

    else if(previousMid && nowDown)
        desiredPosition -= dr;

    dp = desiredPosition - float(actualPosition);

    if(dp != 0) {

        p = 3000 * int(dp / dr + abs(dp) / dp);
        p += t;

        if (smooth) {
            p -= int((pow((dr - abs(dp)) / dr, 2.7) * t) * (abs(dp) / dp));
        } else {
            p += int(1 * (-pow((abs(dp) - dr / 2 - 30) / 8.35, 2) + 3000) * abs(dp) / dp);
            p -= int((pow((dr - abs(dp)) / dr, 3) * t) * (abs(dp) / dp));
        }

        if (abs(p) > 32760 && p != 0)
            p = (abs(p) / p) * 32760;

        if (abs(int(desiredPosition) - actualPosition) > 50)
            indexer.setPower(p);

        else
            indexer.setPower(0);

        printf("pos: %d\n", int(desiredPosition));
        printf("POSITION: %d\n\n", actualPosition);

        printf("s %d\n", p);
        printf("torque: %d\n", t);
    }

}

void stopFLyWheels(){
    if(RFLYWHEEL.getData(VELOCITY) > 200)
        setFlyWheelPwr(-RFLYWHEEL.getData(VELOCITY));
    else
        setFlyWheelPwr(0);
}

int main(){

    imuThread.start(runImuThread);

    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    DJIMotor::sendValues();
    DJIMotor::getFeedback();

    setMotorSettings();

    int actualPosition = indexer.getData(MULTITURNANGLE);
    float desiredPosition = float(int((actualPosition / dr)) * dr) + IOFFSET;
    int t;

    bool previousMid;
    bool nowUp;
    bool nowDown;

    float translationalMultiplier = 1.5;

    int refLoop = 0;
    int yawSetPoint = 0;

    double beyblade;
    double ref_chassis_power;
    int max_power;

    DJIMotor::getFeedback();
    unsigned long loopTimer = us_ticker_read() / 1000;

    while(true){
        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;
            refLoop++;

            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led = !led;
            }

            previousMid = lS == Remote::SwitchState::MID;
            remoteRead();

            nowUp = lS == Remote::SwitchState::UP;
            nowDown = lS == Remote::SwitchState::DOWN;

            actualPosition = indexer.getData(MULTITURNANGLE);
            t = indexer.getData(TORQUE);

            ref_chassis_power = ext_power_heat_data.data.chassis_power;
            max_power = ext_game_robot_state.data.chassis_power_limit;
            //printff("Ref power: %i\n", (int) (ref_chassis_power * 100));

            if (!sticksMoved) {
                chassis.driveXYR({0,0,0});

                if ((prevLS != Remote::SwitchState::UNKNOWN && lS != prevLS ) || (prevRS != Remote::SwitchState::UNKNOWN && rS != prevRS))
                    sticksMoved = true;

                else
                    prevLS = lS, prevRS = rS;

            }else if(rS == Remote::SwitchState::UP){
                //pitch.setPosition((-rY / 2) + 5600);
                yawSetPoint -= int(rX / 10.0);
                yaw.setPosition(yawSetPoint);
                indexerLoop(previousMid, nowUp, nowDown, actualPosition, desiredPosition, t);
                setFlyWheelPwr(16000);
                beyblade = 2000;

            }else{
                yaw.setPower(0);
                //pitch.setPower(0);
                indexer.setPower(0);

                desiredPosition = float(int((actualPosition / dr)) * dr) + IOFFSET;
                stopFLyWheels();
                beyblade = 0;
            }

            chassis.driveTurretRelativePower(ref_chassis_power, max_power,{-lX * 5.0, -lY * 5.0, beyblade},  -yaw.getData(MULTITURNANGLE) * 360.0 / 8192 + 90);
            DJIMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

#pragma clang diagnostic pop

