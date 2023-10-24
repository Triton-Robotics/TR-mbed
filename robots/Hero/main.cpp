#include "main.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define PI 3.14159265

#define dr (8191.0 / 9.0)
#define IOFFSET 30
#define smooth true

DigitalOut led(L26);
DigitalOut led2(L27);
I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);

DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY, "YAW");
DJIMotor indexer(7, CANHandler::CANBUS_1, GIMBLY, "INDEXER");

DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY, "PITCH");
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2,M3508, "RFLYWHEEL");
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2,M3508, "LFLYWHEEL");

Remote::SwitchState prevRS = Remote::SwitchState::UNKNOWN, prevLS = Remote::SwitchState::UNKNOWN;

Thread imuThread;

void runImuThread(){
    chassis.initializeImu();
    while (true) {
        chassis.readImu();
        ThisThread::sleep_for(25);
    }
}

void setFlyWheelPwr(int pwr){
    RFLYWHEEL.setPower(pwr);
    LFLYWHEEL.setPower(-pwr);
}

void stopFLyWheels(){
    if(RFLYWHEEL.getData(VELOCITY) > 200)
        setFlyWheelPwr(-RFLYWHEEL.getData(VELOCITY));
    else
        setFlyWheelPwr(0);
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

        if (abs(int(desiredPosition) - actualPosition) > 50)
            indexer.setPower(p);

        else
            indexer.setPower(0);

//        printf("pos: %d\n", int(desiredPosition));
//        printf("POSITION: %d\n\n", actualPosition);
//
//        printf("s %d %d\n", p, indexer.powerOut);
//        printf("torque: %d\n", t);
    }

}

void pitchSetPosition(){

    int pitchSetPoint = -remote.rightY() / 1.5 + 5660;

    if(pitchSetPoint > 6100)
        pitchSetPoint = 6100;

    else if(pitchSetPoint < 5000)
        pitchSetPoint = 5000;

    pitch.setPosition(pitchSetPoint);
    //printf("%d %d\n", pitchSetPoint, pitch.getData(ANGLE));

}

int calculateDeltaYaw(int ref_yaw, int beforeBeybladeYaw){
    int deltaYaw = beforeBeybladeYaw - ref_yaw;

    if(abs(deltaYaw) > 180){
        if(deltaYaw > 0)
            deltaYaw -= 360;
        else
            deltaYaw += 360;
    }
    return deltaYaw;
}

void setMotorSettings(){

    pitch.setPositionPID(15, 0.2, 10);
    //10, 0.3, 50
    //15, 0.6, 70
    //20, 0.7, 70

    pitch.pidPosition.setIntegralCap(3000);
    pitch.setPositionIntegralCap(10000);
    pitch.useAbsEncoder = true;


    yaw.setPositionPID(50, 0.3, 1);
    yaw.setPositionOutputCap(100000);
    yaw.setPositionIntegralCap(10000);
    yaw.outputCap = 32760;
    yaw.useAbsEncoder = false;

    indexer.setPositionOutputCap(100000);
    indexer.setSpeedOutputCap(1000000);
    indexer.setPositionIntegralCap(100000);
    indexer.setSpeedIntegralCap(100000);
    indexer.outputCap = 32700;

    LFLYWHEEL.outputCap = 16380;
    RFLYWHEEL.outputCap = 16380;

    chassis.setBrakeMode(Chassis::COAST);

}

int main(){

    imuThread.start(runImuThread);

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    setMotorSettings();

    int actualPosition = indexer.getData(MULTITURNANGLE);
    float desiredPosition = float(int((actualPosition / dr)) * dr) + IOFFSET;
    int t;
    int p;

    bool lPreviousMid;
    bool lNowUp;
    bool lNowDown;

    bool rPreviousDown;
    bool rNowMid;
    bool rNowDown;
    bool once = false;
    bool onceT = false;

    DJIMotor::s_getFeedback();
    int refLoop = 0;
    int yawSetPoint;
    int ref_yaw;
    int beforeBeybladeYaw;
    int deltaYaw;
    double rotationalPower = 0;

    double beyblade;
    float chassis_power;
    uint16_t chassis_power_limit;
    unsigned long count = 0;

    unsigned long loopTimer = us_ticker_read();
    unsigned long timeEnd;
    unsigned long timeStart;

    PID yawPID(100, 0.1, 500, 50000, 32760);

    while(true){
        timeStart = us_ticker_read();

        if((timeStart - loopTimer) / 1000 > 25) {
            loopTimer = timeStart;


            if (refLoop >= 5) {
                refLoop = 0;
                refereeThread(&referee);
            }

            if(!once && remote.leftSwitch() == Remote::SwitchState::DOWN){
                once = true;
                yawSetPoint = yaw.getData(MULTITURNANGLE);
            }

            count++;
            refLoop++;

            lPreviousMid    = remote.leftSwitch()   == Remote::SwitchState::MID;
            rPreviousDown   = remote.rightSwitch()  == Remote::SwitchState::DOWN;
            remoteRead();

            rNowDown        = remote.rightSwitch()  == Remote::SwitchState::DOWN;
            rNowMid         = remote.rightSwitch()  == Remote::SwitchState::MID;
            lNowUp          = remote.leftSwitch()   == Remote::SwitchState::UP;
            lNowDown        = remote.leftSwitch()   == Remote::SwitchState::DOWN;

            actualPosition = indexer.getData(MULTITURNANGLE);
            t = indexer.getData(TORQUE);

            chassis_power = ext_power_heat_data.data.chassis_power;
            chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;
            ref_yaw = int(ext_game_robot_pos.data.yaw);

            //printf("yaw: %d pitch %d\n", int(ref_yaw), int((pitch.getData(ANGLE) - 5660) * 360.0 / 8191.0));

            //printf("%d\n", chassis.getHeadingDegrees());
            //printf("%d\n",chassis.getHeadingDegrees());
            //printf("%d %d\n", lX, lY);

            //printf("ref power: %d \n", int(chassis_power * 100));


            if(rPreviousDown){
                if(rNowMid) {
                    yaw.setPower(0);
                }
            }else if(rNowDown){
                beforeBeybladeYaw = chassis.getHeadingDegrees();
            }

            if(remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN){
                desiredPosition = float(int((actualPosition / dr)) * dr) + IOFFSET;
                indexer.setPower(0);
                yaw.setPower(0);
                stopFLyWheels();
                beyblade = 0;
                yaw.setPower(-remote.rightX() * 20);

            }else{
                indexerLoop(lPreviousMid, lNowUp, lNowDown, actualPosition, desiredPosition, t);
                setFlyWheelPwr(16300);

                if(remote.rightSwitch() == Remote::SwitchState::DOWN) {
                    beyblade = 2000;
                    deltaYaw = calculateDeltaYaw(chassis.getHeadingDegrees(), beforeBeybladeYaw);
                    yaw.setPower((int)yawPID.calculatePeriodic(static_cast<float>(deltaYaw),us_ticker_read() - timeEnd));
                }else {
                    beyblade = 0;
                    yaw.setPower(-remote.rightX() * 20);
                }
            }

            pitchSetPosition();
            timeEnd = us_ticker_read();
            chassis.driveTurretRelativePower(chassis_power, chassis_power_limit, {-remote.leftX() * 5.0, -remote.leftY() * 5.0, beyblade}, (yaw.getData(MULTITURNANGLE) - yawSetPoint) * 180.0 / 8191.0 - 180, int(timeEnd - timeStart), rotationalPower);
            //printf("%d %d %d %d\n" , yawSetPoint, yaw.getData(MULTITURNANGLE) - yawSetPoint, int((yaw.getData(MULTITURNANGLE) - yawSetPoint) * 180.0 / 8191.0 - 180), chassis.getHeadingDegrees());
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

#pragma clang diagnostic pop

//
//#include "main.h"
//
//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "EndlessLoop"
//#define PI 3.14159265
//
//#define LOWERBOUND 1000
//#define UPPERBOUND 2000
//
//// CANMotor LF(4,NewCANHandler::CANBUS_1,M3508);
//// CANMotor RF(2,NewCANHandler::CANBUS_1,M3508);
//// CANMotor LB(1,NewCANHandler::CANBUS_1,M3508);
//// CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);
//
//
//I2C i2c(I2C_SDA, I2C_SCL);
//Chassis chassis(1, 2, 3, 4, &i2c);
//DigitalOut led(L26);
//DigitalOut led2(L27);
//
//DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
//DJIMotor pitch(6, CANHandler::CANBUS_1, GIMBLY);
//int pitchval = 0;
//
//DJIMotor indexer(7, CANHandler::CANBUS_1, GIMBLY);
//int indexJamTime = 0;
//int lastJam = 0;
//
//unsigned long cT = 0;
//unsigned long forwardTime = 250;
//unsigned long reverseTime = 300;
//unsigned long totalTime;
//
//bool sticksMoved = false;
//Remote::SwitchState prevRS = Remote::SwitchState::UNKNOWN, prevLS = Remote::SwitchState::UNKNOWN;
//
//DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2,M3508);
//DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2,M3508);
//
//void setFlyWheelPwr(int pwr) {
//    // for (int i = 0; i < 2; i++)
//    //     //flyWheelMotors[i].set(pwr)
//    RFLYWHEEL.setPower(pwr);
//    LFLYWHEEL.setPower(-pwr);
////    LFLYWHEEL.setSpeed(-pwr);
////    RFLYWHEEL.setSpeed(pwr);
//    //printf("L%d R%d\n", LFLYWHEEL.getData(VELOCITY), RFLYWHEEL.getData(VELOCITY));
//}
//
//Thread imuThread;
//
//[[noreturn]] void runImuThread() {
//    chassis.initializeImu();
//    while (true) {
//        chassis.readImu();
//        ThisThread::sleep_for(25);
//    }
//}
//
//int main()
//{
//    imuThread.start(runImuThread);
//    float speedMultiplier = 3;
//    float powMultiplier = 2;
//    float translationalMultiplier = 1.5; // was 3
//    float beybladeSpeedMult = 1;
//
//    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
//
//    pitch.setPositionPID(4, 0.85, 0.15);
//    pitch.pidPosition.setIntegralCap(3000);
//    //pitch.setPositionPID(4, 0.35, 0.35);
//    pitch.setPositionIntegralCap(10000);
//
//    pitch.useAbsEncoder = true;
//    pitch.justPosError = true;
//
//    yaw.setPositionPID(3.5, 0, 0.25);
//    yaw.setPositionIntegralCap(10000);
//    yaw.justPosError = true;
//
//
//    indexer.justPosError = true;
//    indexer.setPositionOutputCap(100000);
//    indexer.setSpeedOutputCap(1000000);
//    indexer.setPositionIntegralCap(100000);
//    indexer.setSpeedIntegralCap(100000);
//    indexer.outCap = 100000;
//
//    LFLYWHEEL.outCap = 16384;
//    RFLYWHEEL.outCap = 16384;
//
//    chassis.setBrakeMode(Chassis::COAST);
//
//    unsigned long loopTimer = us_ticker_read() / 1000;
//
//    int indexJamTime = 0;
//
//    bool strawberryJam = false;
//    int refLoop=0;
//
//    int yawSetPoint = 0;
//
//    DJIMotor::getFeedback();
//    double beybladeSpeed = 2;
//    bool beybladeIncreasing = true;
//
//    double ref_chassis_power;
//    int max_power;
//
//    while (true) {
//
//
//        unsigned long timeStart = us_ticker_read() / 1000;
//        if(timeStart - loopTimer > 25){
//            loopTimer = timeStart;
//            led = !led;
//
//            refLoop++;
//
//            if (refLoop >= 5)
//            {
//                refereeThread(&referee);
//                // printf("thread\n");
//                refLoop = 0;
//                //                 led = ext_power_heat_data.data.chassis_power > 0;
//                //                 printf("%d\n",ext_power_heat_data.data.chassis_power);
//                led2 = !led2;
//            }
//
//            ref_chassis_power = ext_power_heat_data.data.chassis_power;
//            max_power = ext_game_robot_state.data.chassis_power_limit;
//
//            remoteRead();
//
//            if (!sticksMoved) {
//                chassis.driveXYR({0,0,0});
//                if ((prevLS != Remote::SwitchState::UNKNOWN && lS != prevLS ) || (prevRS != Remote::SwitchState::UNKNOWN && rS != prevRS)) {
//                    sticksMoved = true;
//                } else {
//                    prevLS = lS;
//                    prevRS = rS;
//                }
//            } else if(rS == Remote::SwitchState::DOWN){ // All non-serializer motors activated
//                int LFa = lY + lX * translationalMultiplier + rX, RFa = lY - lX * translationalMultiplier - rX, LBa = lY - lX * translationalMultiplier + rX, RBa = lY + lX * translationalMultiplier - rX;
//                chassis.driveFieldRelative({lX / 500.0, lY / 500.0, rX / 500.0});
//
//                pitch.setPosition((-rY / 2) + 5600);
//                // yaw.setSpeed(rX/100);
//                yawSetPoint -= rX / 10.0;
//                yaw.setPosition(yawSetPoint);
//
//
//            }else if(rS == Remote::SwitchState::MID){ //disable all the non-serializer components
//                chassis.driveXYR({0, 0, 0});
//                yaw.setPower(0); pitch.setPower(0);
//            }else if(rS == Remote::SwitchState::UP){ // beyblade mode
//                // chassis.beyblade(lX / 500.0, lY / 500.0, true);
//                // yaw.setPower(0); pitch.setPower(0);
//
//                int LFa = lY + lX * translationalMultiplier + rX, RFa = lY - lX * translationalMultiplier - rX, LBa = lY - lX * translationalMultiplier + rX, RBa = lY + lX * translationalMultiplier - rX;
//                chassis.driveFieldRelative({lX / 500.0, lY / 500.0, 0});
//
//                //pitch.setPosition((rY / 2) + 5600);
//                pitch.setPower(-rY*5);
//                // yaw.setSpeed(rX/100);
//                yawSetPoint -= rX / 5.0;
//                yaw.setPosition(yawSetPoint);
//            }
//
//            if (lS == Remote::SwitchState::UP) {
//                //indexer.setPower(1200);
//                if (rY < 200 && rY > -200) {
//                    indexer.setPower(0);
//                } else {
//                    indexer.setPower(rY * 32);
//                }
////                setFlyWheelPwr(80000);
//                LFLYWHEEL.setPower(-16384); RFLYWHEEL.setPower(16384);
//            }else if(lS == Remote::SwitchState::MID){ //disable serializer
//                //indexer.setSpeed(5);
//                if (rY < 200 && rY > -200) {
//                    indexer.setPower(0);
//                } else {
//                    indexer.setPower(rY * 32);
//                    printf("%d\n", rY * 32);
//                }
////                setFlyWheelPwr(60000);
//                LFLYWHEEL.setPower(-16384); RFLYWHEEL.setPower(16384);
//            }else if(lS == Remote::SwitchState::DOWN){
//                ///////////////////////////////////////////
//                /// THEO SECTION OF CODE
//                ///////////////////////////////////////////
//                //printf("TORQ:%d VEL:%d\n",indexer.getData(TORQUE), indexer.getData(VELOCITY));
//                // if(abs(indexer.getData(TORQUE)) > 100 & abs(indexer.getData(VELOCITY)) < 20){ //jam
//                //     indexJamTime = us_ticker_read() /1000;
//                // }
//                // if(us_ticker_read() / 1000 - indexJamTime < 1000){
//                //     indexer.setPower(-14000); //jam
//                //     //printf("JAMMMMM- ");
//                // }else if(us_ticker_read() / 1000 - indexJamTime < 1500){
//                //     indexer.setPower(9000); //jam
//                //     //printf("POWER FORWARD- ");
//                // }else{
//                //     //indexer.setPower(-900);
//                //     indexer.setSpeed(4500);
//                // }
//                indexer.setPower(0);
//                setFlyWheelPwr(0);
//            }
//            DJIMotor::sendValues();
//        }
//        unsigned long timeEnd = us_ticker_read() / 1000;
//        DJIMotor::getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}
//
//
//
////if(refLoop > 50){
////            //     //refereeThread(&referee);
////            //     refLoop = 0;
////            //     //led = ext_power_heat_data.data.chassis_power > 0;
////            //     //printf("%d\n",ext_power_heat_data.data.chassis_power);
////            }
//////            printf("A %i B %i\n", rS, lS);
////
////            //printf("stik %d swit %d\n", lY, lS);
//
//#pragma clang diagnostic pop
//