#define DEBUG 1

#if DEBUG == 0

#include "main.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define PI 3.14159265

#define LOWERBOUND 1000
#define UPPERBOUND 2000

// CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); 
// CANMotor RF(2,NewCANHandler::CANBUS_1,M3508); 
// CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); 
// CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);


I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);
DigitalOut led(L26);
DigitalOut led2(L27);

DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(6, CANHandler::CANBUS_1, GIMBLY);
int pitchval = 0;

DJIMotor indexer(7, CANHandler::CANBUS_1, GIMBLY);
int indexJamTime = 0;
int lastJam = 0;

unsigned long cT = 0;
unsigned long forwardTime = 250;
unsigned long reverseTime = 300;
unsigned long totalTime;

bool sticksMoved = false;
Remote::SwitchState prevRS = Remote::SwitchState::UNKNOWN, prevLS = Remote::SwitchState::UNKNOWN;

DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2,M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2,M3508);

void setFlyWheelPwr(int pwr) {
    // for (int i = 0; i < 2; i++)
    //     //flyWheelMotors[i].set(pwr)
    RFLYWHEEL.setPower(pwr);
    LFLYWHEEL.setPower(-pwr);
//    LFLYWHEEL.setSpeed(-pwr);
//    RFLYWHEEL.setSpeed(pwr);
    //printf("L%d R%d\n", LFLYWHEEL.getData(VELOCITY), RFLYWHEEL.getData(VELOCITY));
}

Thread imuThread;

[[noreturn]] void runImuThread() {
    chassis.initializeImu();
    while (true) {
        chassis.readImu();
        ThisThread::sleep_for(25);
    }
}

int main()
{
    imuThread.start(runImuThread);
    float speedMultiplier = 3;
    float powMultiplier = 2;
    float translationalMultiplier = 1.5; // was 3
    float beybladeSpeedMult = 1;

    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);

    pitch.setPositionPID(4, 0.85, 0.15);
    pitch.pidPosition.setIntegralCap(3000);
    //pitch.setPositionPID(4, 0.35, 0.35);
    pitch.setPositionIntegralCap(10000);

    pitch.useAbsEncoder = true;
    pitch.justPosError = true;

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

    unsigned long loopTimer = us_ticker_read() / 1000;

    int indexJamTime = 0;

    bool strawberryJam = false;
    int refLoop=0;

    int yawSetPoint = 0;

    DJIMotor::getFeedback();
    double beybladeSpeed = 2;
    bool beybladeIncreasing = true;

    double ref_chassis_power;
    int max_power;

    while (true) {
        

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;
            led = !led;

            refLoop++;

            if (refLoop >= 5)
            {
                refereeThread(&referee);
                // printf("thread\n");
                refLoop = 0;
                //                 led = ext_power_heat_data.data.chassis_power > 0;
                //                 printf("%d\n",ext_power_heat_data.data.chassis_power);
                led2 = !led2;
            }

            ref_chassis_power = ext_power_heat_data.data.chassis_power;
            max_power = ext_game_robot_state.data.chassis_power_limit;

            remoteRead();

            if (!sticksMoved) {
                chassis.driveXYR({0,0,0});
                if ((prevLS != Remote::SwitchState::UNKNOWN && lS != prevLS ) || (prevRS != Remote::SwitchState::UNKNOWN && rS != prevRS)) {
                    sticksMoved = true;
                } else {
                    prevLS = lS;
                    prevRS = rS;
                }
            } else if(rS == Remote::SwitchState::DOWN){ // All non-serializer motors activated
                int LFa = lY + lX * translationalMultiplier + rX, RFa = lY - lX * translationalMultiplier - rX, LBa = lY - lX * translationalMultiplier + rX, RBa = lY + lX * translationalMultiplier - rX;
                chassis.driveFieldRelative({lX / 500.0, lY / 500.0, rX / 500.0});

                pitch.setPosition((-rY / 2) + 5600);
                // yaw.setSpeed(rX/100);
                yawSetPoint -= rX / 10.0;
                yaw.setPosition(yawSetPoint);


            }else if(rS == Remote::SwitchState::MID){ //disable all the non-serializer components
                chassis.driveXYR({0, 0, 0});
                yaw.setPower(0); pitch.setPower(0);
            }else if(rS == Remote::SwitchState::UP){ // beyblade mode
                // chassis.beyblade(lX / 500.0, lY / 500.0, true);
                // yaw.setPower(0); pitch.setPower(0);

                int LFa = lY + lX * translationalMultiplier + rX, RFa = lY - lX * translationalMultiplier - rX, LBa = lY - lX * translationalMultiplier + rX, RBa = lY + lX * translationalMultiplier - rX;
                chassis.driveFieldRelative({lX / 500.0, lY / 500.0, 0});

                //pitch.setPosition((rY / 2) + 5600);
                pitch.setPower(-rY*5);
                // yaw.setSpeed(rX/100);
                yawSetPoint -= rX / 5.0;
                yaw.setPosition(yawSetPoint);
            }

            if (lS == Remote::SwitchState::UP) {
                //indexer.setPower(1200);
                if (rY < 200 && rY > -200) {
                    indexer.setPower(0);
                } else {
                    indexer.setPower(rY * 32);
                }
//                setFlyWheelPwr(80000);
                LFLYWHEEL.setPower(-16384); RFLYWHEEL.setPower(16384);
            }else if(lS == Remote::SwitchState::MID){ //disable serializer
                //indexer.setSpeed(5);
                if (rY < 200 && rY > -200) {
                    indexer.setPower(0);
                } else {
                    indexer.setPower(rY * 32);
                    printf("%d\n", rY * 32);
                }
//                setFlyWheelPwr(60000);
                LFLYWHEEL.setPower(-16384); RFLYWHEEL.setPower(16384);
            }else if(lS == Remote::SwitchState::DOWN){
                ///////////////////////////////////////////
                /// THEO SECTION OF CODE
                ///////////////////////////////////////////
                //printf("TORQ:%d VEL:%d\n",indexer.getData(TORQUE), indexer.getData(VELOCITY));
                // if(abs(indexer.getData(TORQUE)) > 100 & abs(indexer.getData(VELOCITY)) < 20){ //jam
                //     indexJamTime = us_ticker_read() /1000;
                // }
                // if(us_ticker_read() / 1000 - indexJamTime < 1000){
                //     indexer.setPower(-14000); //jam
                //     //printf("JAMMMMM- ");
                // }else if(us_ticker_read() / 1000 - indexJamTime < 1500){
                //     indexer.setPower(9000); //jam
                //     //printf("POWER FORWARD- ");
                // }else{
                //     //indexer.setPower(-900);   
                //     indexer.setSpeed(4500);
                // }
                indexer.setPower(0);
                setFlyWheelPwr(0);
            }
            DJIMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}



//if(refLoop > 50){
//            //     //refereeThread(&referee);
//            //     refLoop = 0;
//            //     //led = ext_power_heat_data.data.chassis_power > 0;
//            //     //printf("%d\n",ext_power_heat_data.data.chassis_power);
//            }
////            printf("A %i B %i\n", rS, lS);
//
//            //printf("stik %d swit %d\n", lY, lS);

#pragma clang diagnostic pop


#else /* DEBUG == 1 ******************************************************************************/

/* Temporary, new hero code - June 2023 */

#include "main.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define PI 3.14159265

#define dr (8191.0 / 9.0)
#define IOFFSET 200
#define smooth true

DigitalOut led(LED1);
I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);

DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor indexer(7, CANHandler::CANBUS_1, GIMBLY);

DJIMotor pitch(6, CANHandler::CANBUS_2, GIMBLY);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2,M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2,M3508);

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

        p = 4000 * int(dp / dr + abs(dp) / dp);
        p += t;

        if (smooth) {
            p -= int((pow((dr - abs(dp)) / dr, 2.7) * t) * (abs(dp) / dp));
        } else {
            p += int(1 * (-pow((abs(dp) - dr / 2 - 30) / 8.35, 2) + 3000) * abs(dp) / dp);
            p -= int((pow((dr - abs(dp)) / dr, 3) * t) * (abs(dp) / dp));
        }

        if (abs(int(desiredPosition) - actualPosition) > 30)
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

    int pitchSetPoint = -rY / 1.5 + 5660;

    if(pitchSetPoint > 6100)
        pitchSetPoint = 6100;

    else if(pitchSetPoint < 5000)
        pitchSetPoint = 5000;

    pitch.setPosition(pitchSetPoint);
    //printf("%d %d\n", pitchSetPoint, pitch.getData(ANGLE));

}

void yawSetPosition(int &yawSetPoint){
    if(abs(rX) > 300)
        yawSetPoint -= int(((abs(rX) - 200) / 6.0) * (rX / abs(rX))); // was div by 10

    yaw.setPosition(yawSetPoint);
}

void setMotorSettings(){

    pitch.justPosError = true;
    pitch.setPositionPID(15, 0.6, 70);
    //10, 0.3, 50
    //15, 0.6, 70
    //20, 0.7, 70

    pitch.pidPosition.setIntegralCap(3000);
    pitch.setPositionIntegralCap(10000);
    pitch.useAbsEncoder = true;

    yaw.justPosError = true;
    yaw.setPositionPID(15, 0.4, 1);
    yaw.setPositionOutputCap(100000);
    yaw.setPositionIntegralCap(10000);
    yaw.outCap = 32760;

    indexer.justPosError = true;
    indexer.setPositionOutputCap(100000);
    indexer.setSpeedOutputCap(1000000);
    indexer.setPositionIntegralCap(100000);
    indexer.setSpeedIntegralCap(100000);
    indexer.outCap = 32700;

    LFLYWHEEL.outCap = 16380;
    RFLYWHEEL.outCap = 16380;

    chassis.setBrakeMode(Chassis::COAST);

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
    int p;

    bool lPreviousMid;
    bool lNowUp;
    bool lNowDown;

    bool rPreviousDown;
    bool rNowMid;
    bool rNowDown;

    int refLoop = 0;
    int yawSetPoint = yaw.getData(MULTITURNANGLE);
    int ref_yaw;
    int beforeBeybladeYaw;
    int deltaYaw;
    double rotationalPower = 0;

    double beyblade;
    float chassis_power;
    uint16_t chassis_power_limit;

    DJIMotor::getFeedback();
    unsigned long loopTimer = us_ticker_read();
    unsigned long timeEnd;
    unsigned long timeStart;

    while(true){
        timeStart = us_ticker_read();

        if((timeStart - loopTimer) / 1000 > 25){
            loopTimer = timeStart;
            refLoop++;
            refereeThread(&referee);

            lPreviousMid    = lS == Remote::SwitchState::MID;
            rPreviousDown   = rS == Remote::SwitchState::DOWN;
            remoteRead();

            rNowDown        = rS == Remote::SwitchState::DOWN;
            rNowMid         = rS == Remote::SwitchState::MID;
            lNowUp          = lS == Remote::SwitchState::UP;
            lNowDown        = lS == Remote::SwitchState::DOWN;

            actualPosition = indexer.getData(MULTITURNANGLE);
            t = indexer.getData(TORQUE);

            chassis_power = ext_power_heat_data.data.chassis_power;
            chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;
            ref_yaw = int(ext_game_robot_pos.data.yaw);

            //printf("yaw: %d pitch %d\n", int(ref_yaw), int((pitch.getData(ANGLE) - 5660) * 360.0 / 8191.0));

            if(rPreviousDown){
                if(rNowMid) {
                    yaw.setPower(0);
                    yawSetPoint = yaw.getData(MULTITURNANGLE);
                }
            }else if(rNowDown){
                beforeBeybladeYaw = ref_yaw;
            }


            //printf("max power: %d ref power: %d \n",  int(chassis_power_limit * 100), int(chassis_power * 100));

            if(rS == Remote::SwitchState::MID || rS == Remote::SwitchState::UNKNOWN){
                desiredPosition = float(int((actualPosition / dr)) * dr) + IOFFSET;
                indexer.setPower(0);
                stopFLyWheels();
                beyblade = 0;
                yawSetPosition(yawSetPoint);

//                p = ref_yaw - yawSetPoint;
//                if(abs(p) > 180 && p != 0)
//                    p -= 360 * (p / abs(p));
//
//                yaw.setPosition(int((yawSetPoint - p) * 8191.0 / 180.0));
//
//                //yaw.setPower(p);
//                printf("%d %d %d %D\n", yawSetPoint, ref_yaw, int((yawSetPoint - p) * 8191.0 / 180.0), yaw.getData(MULTITURNANGLE));

            }else{
                indexerLoop(lPreviousMid, lNowUp, lNowDown, actualPosition, desiredPosition, t);
                setFlyWheelPwr(16300);
                //printf("%d %d\n", LFLYWHEEL.getData(VELOCITY), RFLYWHEEL.getData(VELOCITY));


                if(rS == Remote::SwitchState::DOWN) {
                    beyblade = 2000;
                    deltaYaw = ref_yaw - beforeBeybladeYaw;

                    while(deltaYaw < 0)
                        deltaYaw += 360;

                    yaw.setPower(rotationalPower + pow(deltaYaw * 10, 2));

                }else {
                    beyblade = 0;
                    yawSetPosition(yawSetPoint);
                }

                //printf("%d %d\n", yawSetPoint, yaw.getData(MULTITURNANGLE));
            }
            pitchSetPosition();
            //pitch.setPower(10000);
            timeEnd = us_ticker_read();
            chassis.driveTurretRelativePower(chassis_power, chassis_power_limit, {-lX * 5.0, -lY * 5.0, beyblade}, ref_yaw + 80, int(timeEnd - timeStart), rotationalPower);
            DJIMotor::sendValues();
        }
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

#pragma clang diagnostic pop

#endif // for DEBUG
