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
DigitalOut led(LED1);

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
    //     //flyWheelMotors[i].set(pwr);
    // RFLYWHEEL.setPower(pwr);
    // LFLYWHEEL.setPower(-pwr);
    LFLYWHEEL.setSpeed(-pwr);
    RFLYWHEEL.setSpeed(pwr);
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

    while (true) {
        led = !led;
        remoteRead();

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            refLoop++;

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