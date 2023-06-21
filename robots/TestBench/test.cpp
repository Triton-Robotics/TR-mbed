//#include "main.h"
//#include "mbed.h"
//
//#define dr (8191.0 / 9.0)
//#define smooth true
//
///*
// * if smooth is true, balls will smoothly come out of serializer with no double feed,
// * but has a low chance of getting stuck around 1/50
// *
// * if smooth is false, balls will come out of serializer with a higher chance of double feed,
// * but there is virtually no chance of balls getting stuck
// *
// * this setting is to be decided as we finish the feeding system
// *
// */
//
//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "EndlessLoop"
//
//DJIMotor indexer(7, CANHandler::CANBUS_1, GIMBLY);
//DigitalOut led(LED1);
//
//
//int main(){
//
//    unsigned long loopTimer = us_ticker_read() / 1000;
//    bool previousMid;
//    bool nowUp;
//    bool nowDown;
//
//    //remote.unfiltered = true;
//    indexer.justPosError = true;
//    indexer.setPositionOutputCap(100000);
//    indexer.setSpeedOutputCap(1000000);
//    indexer.setPositionIntegralCap(100000);
//    indexer.setSpeedIntegralCap(100000);
//    indexer.outCap = 100000;
//
//    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
//    DJIMotor::sendValues();
//    DJIMotor::getFeedback();
//
//    int actualPosition = indexer.getData(MULTITURNANGLE);
//    float desiredPosition = float(int((actualPosition / dr)) * dr) + 200;
//    int p;
//    int t;
//    float dp;
//
//    printf("------------------------------------\n");
//    printf("[i] %d\n", int(actualPosition));
//    printf("[p] %d\n", int(desiredPosition));
//
//    while (true) {
//        unsigned long timeStart = us_ticker_read() / 1000;
//
//        if(timeStart - loopTimer > 25){
//            loopTimer = timeStart;
//
//            previousMid = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID);
//            remoteRead();
//
//            nowUp = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP);
//            nowDown = bool(remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN);
//
//            led = !led;
//            actualPosition = indexer.getData(MULTITURNANGLE);
//            t = indexer.getData(TORQUE);
//
//            if(rS == Remote::SwitchState::DOWN) {
//                if(previousMid && nowUp)
//                    desiredPosition += dr;
//
//                else if(previousMid && nowDown)
//                    desiredPosition -= dr;
//
//                dp = desiredPosition - float(actualPosition);
//                p = 3000 * int(dp / dr + abs(dp) / dp);
//                p += t;
//
//                if(smooth){
//                    p -= int((pow((dr - abs(dp)) / dr, 2.7) * t) * (abs(dp) / dp));
//                }else{
//                    p += int(1 * (-pow((abs(dp) - dr/2 - 30) / 8.35, 2) + 3000) * abs(dp) / dp);
//                    p -= int((pow((dr - abs(dp)) / dr, 3) * t) * (abs(dp) / dp));
//                }
//
//                if (abs(p) > 32760)
//                    p = (abs(p) / p) * 32760;
//
//                if(abs(int(desiredPosition) - actualPosition) > 50)
//                    indexer.setPower(p);
//
//                else
//                    indexer.setPower(0);
//
//                printf("s %d\n", p);
//                printf("torque: %d\n", t);
//
//            }else if(rS == Remote::SwitchState::MID) {
//                desiredPosition = float(int((actualPosition / dr)) * dr) + 200;
//                indexer.setPosition(int(desiredPosition));
//                indexer.setPower(0);
//
//            }else
//                indexer.setPower(0);
//
//            printf("pos: %d\n", int(desiredPosition));
//            printf("POSITION: %d\n\n", actualPosition);
//
//            DJIMotor::sendValues();
//        }
//
//        DJIMotor::getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}
//#pragma clang diagnostic pop



//
//#include "main.h"
//
//#pragma clang diagnostic push
//#pragma ide diagnostic ignored "EndlessLoop"
//#define PI 3.14159265
//
//#define dr (8191.0 / 9.0)
//#define IOFFSET 200
//#define smooth true
//
//#define LOWERBOUND 1000
//#define UPPERBOUND 2000
//
//DigitalOut led(LED1);
//I2C i2c(I2C_SDA, I2C_SCL);
//Chassis chassis(1, 2, 3, 4, &i2c);
//
//DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
//DJIMotor pitch(6, CANHandler::CANBUS_1, GIMBLY);
//DJIMotor indexer(7, CANHandler::CANBUS_1, GIMBLY);
//DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2,M3508);
//DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2,M3508);
//
//Remote::SwitchState prevRS = Remote::SwitchState::UNKNOWN, prevLS = Remote::SwitchState::UNKNOWN;
//
//Thread imuThread;
//bool sticksMoved = false;
//
//void setFlyWheelPwr(int pwr){
//    // for (int i = 0; i < 2; i++)
//    //     //flyWheelMotors[i].set(pwr);
//    // RFLYWHEEL.setPower(pwr);
//    // LFLYWHEEL.setPower(-pwr);
//    LFLYWHEEL.setSpeed(-pwr);
//    RFLYWHEEL.setSpeed(pwr);
//    //printf("L%d R%d\n", LFLYWHEEL.getData(VELOCITY), RFLYWHEEL.getData(VELOCITY));
//}
//
//[[noreturn]] void runImuThread(){
//    chassis.initializeImu();
//    while (true) {
//        chassis.readImu();
//        ThisThread::sleep_for(25);
//    }
//}
//
//void setMotorSettings(){
//
//    pitch.setPositionPID(4, 0.85, 0.15);
//    pitch.pidPosition.setIntegralCap(3000);
//    pitch.setPositionIntegralCap(10000);
//    pitch.useAbsEncoder = true;
//    pitch.justPosError = true;
//
//    yaw.setPositionPID(3.5, 0, 0.25);
//    yaw.setPositionIntegralCap(10000);
//    yaw.justPosError = true;
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
//}
//
//void indexerLoop(bool &previousMid, bool &nowUp, bool &nowDown, int &actualPosition, float &desiredPosition, int &t){
//
//    float dp;
//    int p;
//
//    if(previousMid && nowUp)
//        desiredPosition += dr;
//
//    else if(previousMid && nowDown)
//        desiredPosition -= dr;
//
//    dp = desiredPosition - float(actualPosition);
//    p = 3000 * int(dp / dr + abs(dp) / dp);
//    p += t;
//
//    if(smooth){
//        p -= int((pow((dr - abs(dp)) / dr, 2.7) * t) * (abs(dp) / dp));
//    }else{
//        p += int(1 * (-pow((abs(dp) - dr/2 - 30) / 8.35, 2) + 3000) * abs(dp) / dp);
//        p -= int((pow((dr - abs(dp)) / dr, 3) * t) * (abs(dp) / dp));
//    }
//
//    if (abs(p) > 32760)
//        p = (abs(p) / p) * 32760;
//
//    if(abs(int(desiredPosition) - actualPosition) > 50)
//        indexer.setPower(p);
//
//    else
//        indexer.setPower(0);
//
//    printf("pos: %d\n", int(desiredPosition));
//    printf("POSITION: %d\n\n", actualPosition);
//
//    printf("s %d\n", p);
//    printf("torque: %d\n", t);
//
//}
//
//int main(){
//
//    imuThread.start(runImuThread);
//
//    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
//    DJIMotor::sendValues();
//    DJIMotor::getFeedback();
//
//    setMotorSettings();
//
//    int actualPosition = indexer.getData(MULTITURNANGLE);
//    float desiredPosition = float(int((actualPosition / dr)) * dr) + IOFFSET;
//    int t;
//
//    bool previousMid;
//    bool nowUp;
//    bool nowDown;
//
//    float translationalMultiplier = 1.5;
//
//    int refLoop = 0;
//    int yawSetPoint = 0;
//
//    DJIMotor::getFeedback();
//    unsigned long loopTimer = us_ticker_read() / 1000;
//
//    while(true){
//        unsigned long timeStart = us_ticker_read() / 1000;
//        if(timeStart - loopTimer > 25){
//            loopTimer = timeStart;
//            refLoop++;
//
//            led = !led;
//
//            previousMid = lS == Remote::SwitchState::MID;
//            remoteRead();
//
//            nowUp = lS == Remote::SwitchState::UP;
//            nowDown = lS == Remote::SwitchState::DOWN;
//
//            actualPosition = indexer.getData(MULTITURNANGLE);
//            t = indexer.getData(TORQUE);
//
//            if (!sticksMoved) {
//                chassis.driveXYR({0,0,0});
//
//                if ((prevLS != Remote::SwitchState::UNKNOWN && lS != prevLS ) || (prevRS != Remote::SwitchState::UNKNOWN && rS != prevRS))
//                    sticksMoved = true;
//
//                else
//                    prevLS = lS, prevRS = rS;
//
//            }else if(rS == Remote::SwitchState::DOWN){ // All non-serializer motors activated
//                chassis.driveFieldRelative({lX / 500.0, lY / 500.0, rX / 500.0});
//
//                pitch.setPosition((-rY / 2) + 5600);
//                // yaw.setSpeed(rX/100);
//                yawSetPoint -= int(rX / 10.0);
//                yaw.setPosition(yawSetPoint);
//                indexerLoop(previousMid, nowUp, nowDown, actualPosition, desiredPosition, t);
//                setFlyWheelPwr(0);
//
//            }else if(rS == Remote::SwitchState::MID){ //disable all the non-serializer components
//                yaw.setPower(0);
//                pitch.setPower(0);
//                indexer.setPower(0);
//                chassis.driveXYR({0, 0, 0});
//
//                desiredPosition = float(int((actualPosition / dr)) * dr) + IOFFSET;
//                setFlyWheelPwr(0);
//
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
//            DJIMotor::sendValues();
//        }
//        unsigned long timeEnd = us_ticker_read() / 1000;
//        DJIMotor::getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}
//
////if(refLoop > 50){
////            //     //refereeThread(&referee);
////            //     refLoop = 0;
////            //     //led = ext_power_heat_data.data.chassis_power > 0;
////            //     //printf("%d\n",ext_power_heat_data.data.chassis_power);
////            }
////            printf("A %i B %i\n", rS, lS);
////
////            //printf("stik %d swit %d\n", lY, lS);
//
//#pragma clang diagnostic pop