#include "main.h"
#include "Infantry.h"
//#include <jetson.h>
#include <cstdlib>
#include <commands/RamseteCommand.h>
#include "communications/DJIRemote.h"
#include "mbed.h"
//#include " COMPONENT_SD/include/SD/SDBlockDevice.h"
//#include "storage/blockdevice/COMPONENT_SD/include/SD/SDBlockDevice.h"
//#include "SDBlockDevice.h"
//#include "storage/blockdevice/COMPONENT_SD/source/SDBlockDevice.cpp"

//SDBlockDevice sd(MBED_CONF_SD_SPI_MOSI, MBED_CONF_SD_SPI_MISO, MBED_CONF_SD_SPI_CLK, MBED_CONF_SD_SPI_CS);

#define PI 3.14159265

#define LOWERBOUND 1000
#define UPPERBOUND 2000

// CANMotor LF(4,NewCANHandler::CANBUS_1,M3508);
// CANMotor RF(2,NewCANHandler::CANBUS_1,M3508);
// CANMotor LB(1,NewCANHandler::CANBUS_1,M3508);
// CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);

Chassis chassis(1, 2, 3, 4);
RamseteCommand command(
        Pose2D(0, 0, 0), Pose2D(20, 20, 0), 2, &chassis);
DigitalOut led(LED1);



DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY);
int pitchval = 0;

DJIMotor indexer(7, CANHandler::CANBUS_1, C610);
int indexJamTime = 0;
int lastJam = 0;

unsigned long cT = 0;
unsigned long forwardTime = 250;
unsigned long reverseTime = 300;
unsigned long totalTime;

bool sticksMoved = false;
Remote::SwitchState prevRS = Remote::SwitchState::UNKNOWN;
Remote::SwitchState prevLS = Remote::SwitchState::UNKNOWN;

DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_1, M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_1, M3508);
DJIMotor flyWheelMotors[] = {RFLYWHEEL, LFLYWHEEL};

void setFlyWheelSpeed(int speed) {
        LFLYWHEEL.setSpeed(-speed);
        RFLYWHEEL.setSpeed(speed);
}

Thread imuThread;

//double getPitchAngle(geometry_msgs__msg__Vector3Stamped jetsonAngles) {
//    return asin(-jetsonAngles.vector.y);
//}
//
//double getYawAngle(geometry_msgs__msg__Vector3Stamped jetsonAngles) {
//    return atan2(jetsonAngles.vector.x, jetsonAngles.vector.z);
//}



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

//    Jetson::init();

    // LB.setSpeedPID(1.75, 0.351, 5.63);
    // RF.setSpeedPID(1.073, 0.556, 0);
    // RB.setSpeedPID(1.081, 0.247, 0.386);
    // LF.setSpeedPID(.743, 0.204, 0.284);
    pitch.setPositionPID(4.2, 0.11, 0.42);
//    pitch.setPositionPID(0, 0, 0);
    pitch.setPositionIntegralCap(10000);
    LFLYWHEEL.setSpeedPID(1, 0, 0);
    RFLYWHEEL.setSpeedPID(1, 0, 0);

    pitch.useAbsEncoder = 1;
    pitch.justPosError = 1;

    yaw.setPositionPID(3.5, 0, 0.25);
    yaw.setPositionIntegralCap(10000);
    yaw.useAbsEncoder = 0;
    yaw.justPosError = 1;

    indexer.setSpeedPID(1.94, 0.002, 0.166);
    indexer.setSpeedIntegralCap(500000);

    chassis.setBrakeMode(Chassis::COAST);

    command.initialize();

    unsigned long loopTimer = us_ticker_read() / 1000;

    int indexJamTime = 0;

    bool strawberryJam = false;
    int refLoop=0;

    int yawSetpoint = 0;
    

    DJIMotor::getFeedback();
    double beybladeSpeed = 2;
    bool beybladeIncreasing = true;

    int counter = 0;

//    printf("Hello World!\n");
//    DIR *dir;
//    struct dirent *ent;
//    if ((dir = opendir ("/")) != NULL) {
//        /* print all the files and directories within directory */
//        while ((ent = readdir (dir)) != NULL) {
//            printf("%s\n", ent->d_name);
//        }
//        closedir (dir);
//    } else {
//        /* could not open directory */
//        printf("Cannot list files!\n");
//    }

    while (true) {

//        printf("Pitch angle: %i\n", (int) pitch.getData(ANGLE));
//        printf("Yaw motor angle: %i\n", (int) yaw.getData(MULTITURNANGLE));
        chassis.printMotorAngle();

//        printf("Pitch: %i\n", (int) pitch.getData((ANGLE)));
//        printf("Yaw: %i\n", (int) yaw.getData((ANGLE)));
//        printf("Speed: %i  %i\n", (int) pitch.getData(VELOCITY), (int) yaw.getData(VELOCITY));
//        printf("Powerout: %i %i \n", (int) pitch.powerOut, (int) yaw.powerOut);

//    if (0 != sd.init()) {
//        printf("Init failed \n");
//    }
//    printf("Reading size!\n");
//    printf("sd size: %llu\n",         sd.size());
//    printf("sd read size: %llu\n",    sd.get_read_size());
//    printf("sd program size: %llu\n", sd.get_program_size());
//    printf("sd erase size: %llu\n",   sd.get_erase_size());

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            led = !led;
            loopTimer = timeStart;
            led = !led;
            remoteRead();

//            if (counter >= 10) {
//                Jetson::update(timeStart - loopTimer);
////        Jetson::odom.translation.x += 0.1;
//                printf("CV x %f\n", Jetson::cv.vector.x);
//                printf("CV y %f\n", Jetson::cv.vector.y);
//                counter = 0;
//            } else {
//                counter++;
//            }


            remoteRead();

            chassis.periodic();
//            printf("Time: %i\n", (int) (timeStart / 1000));

//            printf("Rs: %i\n", (int) rS);


//            printf("Lx: %i\n", (int) lX);
//            printf("Ly: %i\n", (int) lY);
//            printf("Rx: %i\n", (int) rX);
//            printf("Ry: %i\n", (int) rY);

            // refLoop++;
            // if(refLoop > 25){
            //     //refereeThread();
            //     refLoop = 0;
            //     //led = ext_power_heat_data.data.chassis_power > 0;
            //     //printf("%d\n",ext_power_heat_data.data.chassis_power);
            // }
//            printf("A %i B %i\n", rS, lS);
            if (!sticksMoved) {
//                printf("NOT MOVED!\n");
                chassis.driveXYR({0,0,0});
                if ((prevLS != Remote::SwitchState::UNKNOWN && lS != prevLS )
                || (prevRS != Remote::SwitchState::UNKNOWN  && rS != prevRS)) {
                    sticksMoved = true;
                } else {
                    prevLS = lS;
                    prevRS = rS;
                }
            } else if(rS == Remote::SwitchState::DOWN){ // All non-serializer motors activated
                int LFa = lY + lX*translationalmultiplier + rX, RFa = lY - lX*translationalmultiplier - rX, LBa = lY - lX*translationalmultiplier + rX, RBa = lY + lX*translationalmultiplier - rX;
//                printf("STICKS: %i %i %i\n", lX, lY, rX);
//                if (chassis.testDataIndex < 300) {
//                    chassis.driveFieldRelative({lX * 5.0, lY * 5.0, 0});
                    chassis.driveTurretRelative({lX * 5.0, lY * 5.0, 0}, yaw.getData(MULTITURNANGLE) * 360.0 / 8192);
//                    chassis.driveFieldRelative(0, 4096, 0);
                    chassis.periodic();

//                    double setpoint = getPitchAngle(Jetson::cv) * 4096 / PI + 6715;
//                    printf("Setpoint: %i\n", (int) setpoint);
//                    pitch.setPosition(setpoint);

                pitch.setPosition((rY / 2) + 1500);


//                 yaw.setSpeed(rX/100);

//                yawSetpoint = (4096 * getYawAngle(Jetson::cv) / PI - yaw.getData(ANGLE));
//                yawSetpoint = 0;
//                yawSetpoint -= rX / 10.0;
//                yawSetpoint -= rX / 10.0;
//                    if (!command.isFinished()) {
//                        printf("running command!\n");
//                        command.execute();
//                    }

//                } else {
//                    chassis.driveXYR(0, 0, 0);
//                        printf("ESTIMATE VS TIME:\n");
//                        for (auto & i : chassis.testData) {
//                            printf("%i\t%i\n", i[0], i[1]);
//                        }
//                        printf("\n\n\nMULTITURN VS TIME:\n");
//                        for (auto & i : chassis.testData) {
//                            printf("%i\t%i\n", i[0], i[2]);
//                        }
//                        printf("\n\n\nVELOCITY VS TIME::\n");
//                        for (auto & i : chassis.testData) {
//                            printf("%i\t%i\n", i[0], i[3]);
//                        }
//                }
//                printf("Angle: %i\n", (int) pitch.getData(ANGLE));
//                pitch.setPosition((rY * 0.9) + 6400);
//                printf("Pitch angle: %i\n", (int) pitch.getData(ANGLE));
//                printf("Pitch power: %i\n", (int) pitch.powerOut);
//                printf("Yaw motor angle: %i\n", (int) yaw.getData(MULTITURNANGLE));
//                printf("Yaw power: %i\n", (int) yawgetData(MULTI.powerOut);
//                printf("Flywheel out: %i\n", (int) LFLYWHEEL.powerOut);
//                printf("Flywheel speed: %i\n", (int) LFLYWHEEL.getData(VELOCITY));
//                printf("Flywheel pos: %i\n", (int) LFLYWHEEL.getData(ANGLE));
//                pitch.setPower((int) (rY * 3));
//                printf("Setting power: %i\n", (int) pitch.powerOut);
                // yaw.setSpeed(rX/100);
//                yawSetpoint -= rX / 10.0;
//                yawSetpoint = -chassis.getHeadingDegrees() * 8192 / 360 + yaw.getData(MULTITURNANGLE) - 2 * rX;
                yaw.setPosition(yawSetpoint);

            }else if(rS == Remote::SwitchState::MID){ //disable all the non-serializer components
//                chassis.driveXYR(0,0,0);
                chassis.driveFieldRelative({0, 0, 0});
                 yaw.setPower(0); pitch.setPower(0);
            }else if(rS == Remote::SwitchState::UP ){ // beyblade mode
                chassis.beyblade(lX * 5.0, lY * 5.0, yaw.getData(MULTITURNANGLE) * 360.0 / 8192, false);
                yawSetpoint = -chassis.getHeadingDegrees() * 8192 / 360 + yaw.getData(MULTITURNANGLE) - 2 * rX;
                yaw.setPosition(yawSetpoint);
//                yaw.setPower(0); pitch.setPower(0);
            }

            if (lS == Remote::SwitchState::UP) {
                //indexer.setPower(1200);
                indexer.setSpeed(2000);
//                printf("Indexer data: %i %i %i\n", (int) (indexer.powerOut), (int) (indexer.getData(ANGLE)), (int) indexer.getData(VELOCITY));
                setFlyWheelSpeed(20000);
//               if (timeStart / 100 == 0) {
//                   printf("Angle: %i\n", (int) 0);
//                    chassis.printMotorAngle();
//               }
            }else if(lS == Remote::SwitchState::MID){ //disable serializer
                indexer.setPower(0);
                setFlyWheelSpeed(0);
            }else if(lS == Remote::SwitchState::DOWN) {
                setFlyWheelSpeed(0);
                ///////////////////////////////////////////
                /// THEO SECTION OF CODE
                ///////////////////////////////////////////
                //printf("TORQ:%d VEL:%d\n",indexer.getData(TORQUE), indexer.getData(VELOCITY));
                if(abs(indexer.getData(TORQUE)) > 100 & abs(indexer.getData(VELOCITY)) < 20){ //jam
                    indexJamTime = us_ticker_read() /1000;
                }
                if(us_ticker_read() / 1000 - indexJamTime < 1000){
                    indexer.setPower(-300); //jam
                    //printf("JAMMMMM- ");
                }else if(us_ticker_read() / 1000 - indexJamTime < 1500){
//                    indexer.setPower(3000); //jam
                  indexer.setPower(0);
                    //printf("POWER FORWARD- ");
                }else{
                    //indexer.setPower(-900);
                    indexer.setSpeed(650);
                }
            }
            DJIMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
