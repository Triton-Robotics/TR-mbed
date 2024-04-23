#include "main.h"
#include "Infantry.h"
// #include <jetson.h>
#include <cstdlib>
// #include <commands/RamseteCommand.h>
#include <iostream>
// #include " COMPONENT_SD/include/SD/SDBlockDevice.h"
// #include "storage/blockdevice/COMPONENT_SD/include/SD/SDBlockDevice.h"
// #include "SDBlockDevice.h"
// #include "storage/blockdevice/COMPONENT_SD/source/SDBlockDevice.cpp"

// SDBlockDevice sd(MBED_CONF_SD_SPI_MOSI, MBED_CONF_SD_SPI_MISO, MBED_CONF_SD_SPI_CLK, MBED_CONF_SD_SPI_CS);

#define PI 3.14159265

#define LOWERBOUND 1000
#define UPPERBOUND 2000

// add radius measurement here
#define RADIUS 0.5

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem chassis(1, 2, 3, 4, imu, RADIUS);
DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY);  // right
DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left
DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap

// RamseteCommand command(
//     Pose2D(0, 0, 0), Pose2D(20, 20, 0), 2, &chassis);
DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

void setFlyWheelSpeed(int speed)
{
    LFLYWHEEL.setSpeed(-speed);
    RFLYWHEEL.setSpeed(speed);
}

int calculateDeltaYaw(int ref_yaw, int beforeBeybladeYaw)
{
    int deltaYaw = beforeBeybladeYaw - ref_yaw;

    if (abs(deltaYaw) > 180)
    {
        if (deltaYaw > 0)
            deltaYaw -= 360;
        else
            deltaYaw += 360;
    }
    return deltaYaw;
}

Thread imuThread;

// double getPitchAngle(geometry_msgs__msg__Vector3Stamped jetsonAngles) {
//     return asin(-jetsonAngles.vector.y);
// }
//
// double getYawAngle(geometry_msgs__msg__Vector3Stamped jetsonAngles) {
//     return atan2(jetsonAngles.vector.x, jetsonAngles.vector.z);
// }

void runImuThread()
{
    chassis.initializeImu();
    while (true)
    {
        chassis.readImu();
        ThisThread::sleep_for(25);
    }
}

void pitchSetPosition()
{

    int pitchSetPoint = (-remote.rightY() * 0.5) + 6200;

    /* TODO: test min and max pitch position */

    // if(pitchSetPoint > 9000)
    //     pitchSetPoint = 9000;

    // else if(pitchSetPoint < 5000)
    //     pitchSetPoint = 5000;

    pitch.setPosition(pitchSetPoint);
    // pitch2.setPower(-pitch.powerOut);
    // printf("%d.%d %d\n", pitchSetPoint, pitch.getData(ANGLE), pitch.powerOut);
}

int main()
{

    imuThread.start(runImuThread);
    float speedmultiplier = 3;
    float powmultiplier = 2;
    float translationalmultiplier = 1.5; // was 3
    float beybladespeedmult = 1;

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);

    pitch.setPositionPID(17.3, 0.03, 8.5); // 12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
    pitch.setPositionIntegralCap(60000);
    pitch.setPositionOutputCap(100000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;

    LFLYWHEEL.setSpeedPID(1, 0, 0);
    RFLYWHEEL.setSpeedPID(1, 0, 0);

    // yaw.setPositionPID(10, 0, 5.5);
    // yaw.setPositionIntegralCap(10000);
    yaw.setPositionPID(10.5, 0.2, 4.4);
    yaw.setPositionIntegralCap(10000);
    yaw.useAbsEncoder = false;

    PID yawIMU(200.0, 0.1, 150, 20000, 8000); // 7.0,0.02,15.0,20000,8000

    gearSwap.setPositionPID(4.0, 0, 0.5);
    gearSwap.setPositionIntegralCap(10000);

    indexer.setSpeedPID(3.1, 0.07, 4);
    indexer.setSpeedIntegralCap(8000);

    chassis.setBrakeMode(ChassisSubsystem::BrakeMode::COAST);

    // command.initialize();

    unsigned long loopTimer = us_ticker_read();

    int indexJamTime = 0;
    bool strawberryJam = false;
    int refLoop = 0;
    int ref_yaw;

    int yawSetPoint = int(-ext_game_robot_pos.data.yaw * 8192 / 360);
    double rotationalPower = 0;

    DJIMotor::s_getFeedback();
    double beybladeSpeed = 2;
    bool beybladeIncreasing = true;

    float chassis_power;
    uint16_t chassis_power_limit;
    uint16_t ref_chassis_temp1;
    uint16_t ref_chassis_temp2;
    uint16_t heatMax1;
    uint16_t heatMax2;

    int counter = 0;

    unsigned long lastTime = 0;
    unsigned long yawTime = us_ticker_read();
    unsigned long burstTimestamp = 0;

    while (true)
    {
        unsigned long timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25)
        {
            led = !led;
            loopTimer = timeStart;
            remoteRead();
            chassis.periodic();

            refLoop++;
            if (refLoop >= 5)
            {
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
            }

            chassis_power = ext_power_heat_data.data.chassis_power;
            chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;

            ref_chassis_temp1 = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
            ref_chassis_temp2 = ext_power_heat_data.data.shooter_id2_17mm_cooling_heat;

            heatMax1 = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit;
            heatMax2 = ext_game_robot_state.data.shooter_id2_17mm_cooling_limit;

            ref_yaw = int(ext_game_robot_pos.data.yaw);

            // printff("%d\n",chassis.getHeadingDegrees());
            // printf("%d %d\n", lX, rX);

            if (remote.rightSwitch() == Remote::SwitchState::UP)
            { // All non-serializer motors activated
                // led3 = 1;
                // int LFa = remote.leftY() + remote.leftX() * translationalmultiplier + remote.rightX();
                // int RFa = remote.leftY() - remote.leftX() * translationalmultiplier - remote.rightX();
                // int LBa = remote.leftY() - remote.leftX() * translationalmultiplier + remote.rightX();
                // int RBa = remote.leftY() + remote.leftX() * translationalmultiplier - remote.rightX();

                // unsigned long time = us_ticker_read();
                // chassis.driveTurretRelativePower(chassis_power, chassis_power_limit, {remote.leftX() * 5.0, remote.leftY() * 5.0, 0}, yaw.getData(ANGLE) * 360.0 / 8192 + 180, int(time - lastTime), rotationalPower);

                // lastTime = time;
                // pitchSetPosition();

                // // yawSetPoint += rX / 36;
                // // yaw.setPosition(-chassis.getHeadingDegrees() * 8192 / 360 + yaw.getData(MULTITURNANGLE) - yawSetPoint);

                // yaw.setPower(-remote.rightX() * 10);

                double scalar = 1;
                double jx = remote.leftX() / 660.0 * scalar;
                double jy = remote.leftY() / 660.0 * scalar;
                double jr = remote.rightX() / 660.0 * scalar;

                double tolerance = 0.05;
                jx = (abs(jx) < tolerance) ? 0 : jx;
                jy = (abs(jy) < tolerance) ? 0 : jy;
                jr = (abs(jr) < tolerance) ? 0 : jr;

                chassis.setSpeedFF_Ks(0.065);
                chassis.setChassisSpeeds({jx * chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * chassis.m_OmniKinematicsLimits.max_Vel,
                                          -jr * chassis.m_OmniKinematicsLimits.max_vOmega},
                                         ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);

                chassis.periodic();

                unsigned long time = us_ticker_read();

                // printff("Yaw:%d Pitch%d\n",(int)((double)(yaw>>MULTITURNANGLE) * 360 / 8192),(int)((double)((pitch>>MULTITURNANGLE) - 6000) * 360 / 8192));
                // printff("y:%d p:%d\n",yaw>>MULTI,pitch>>ANGLE);
            }
            else if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN || remote.rightSwitch() == Remote::SwitchState::DOWN)
            { // disable all the non-serializer components
                led3 = 0;
                chassis.setWheelPower({0, 0, 0, 0});
                yaw.setPower(0);
                pitch.setPower(0);
                pitch2.setPower(0);
                yawSetPoint = int(ext_game_robot_pos.data.yaw);
                // printff("whe%d\n",Wh);
                pitchSetPosition();
            }
            yawTime = us_ticker_read();
            if (remote.leftSwitch() == Remote::SwitchState::MID)
            {
                gearSwap.setPower(-1500);
                double mps = ext_game_robot_state.data.shooter_id1_17mm_speed_limit * 0.9;
                double rpm = 60 * (mps / 0.03) / (3.14159 * 2);
                setFlyWheelSpeed(rpm);
                indexer.setPower(0);
                // printf("Ref temp barrel 1: %i Ref temp barrel 2: %i\n", (int)ref_chassis_temp1, (int) ref_chassis_temp2);

                // printf("MMAX barrel temp currently # 1: %i MAX barrel temp currently # 2: %i\n", (int)heatMax1, (int)heatMax2);
                //
                //                if (ref_chassis_temp1 < heatMax1) // not finish yet
                //                {
                //                    // indexer.setPower(1200);
                //                    indexer.setSpeed(2000);
                //                    // printf("Indexer data: %i %i %i\n", (int) (indexer.powerOut), (int) (indexer.getData(ANGLE)), (int) indexer.getData(VELOCITY));
                //                    setFlyWheelSpeed(20000);
                //                }
                //                else
                //                {
                //                    indexer.setSpeed(0);
                //                    setFlyWheelSpeed(0);
                //                }

                //               if (timeStart / 100 == 0) {
                //                   printf("Angle: %i\n", (int) 0);
                //                    chassis.printMotorAngle();
                //               }
                burstTimestamp = us_ticker_read();
            }
            else if (remote.leftSwitch() == Remote::SwitchState::DOWN || remote.leftSwitch() == Remote::SwitchState::UNKNOWN)
            { // disable serializer
                gearSwap.setPower(-1500);
                indexer.setPower(0);
                setFlyWheelSpeed(0);
            }
            else if (remote.leftSwitch() == Remote::SwitchState::UP)
            {
                gearSwap.setPower(-1500);
                double mps = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;
                double rpm = 60 * (mps / 0.03) / (3.14159 * 2);
                setFlyWheelSpeed(rpm); // was 0 * 0/

                // printf("TORQ:%d VEL:%d\n",indexer.getData(TORQUE), indexer.getData(VELOCITY));
                // if (abs(indexer.getData(TORQUE)) > 100 & abs(indexer.getData(VELOCITY)) < 20)
                // { // jam
                //     indexJamTime = us_ticker_read() / 1000;
                // }
                // if (us_ticker_read() / 1000 - indexJamTime < 1000)
                // {
                //     indexer.setPower(-300); // jam
                //     // printf("JAMMMMM- ");
                // }
                // else if (us_ticker_read() / 1000 - indexJamTime < 1500)
                // {
                //     //                    indexer.setPower(3000); //jam
                //     indexer.setPower(0);
                //     // printf("POWER FORWARD- ");
                // }
                // else
                // {
                //     // indexer.setPower(-900);
                //     indexer.setSpeed(1000); // was 650
                // }
                // indexer.setSpeed(3);
                if ((us_ticker_read() - burstTimestamp) / 1000 < 99)
                {
                    indexer.setPower(8000);
                }
                else
                {
                    indexer.setPower(0);
                }
                // printff("%d\n", indexer.powerOut);
                printff("v:%d\n", indexer.getData(VELOCITY));
                if (heatMax1 - ref_chassis_temp1 < 60)
                {
                    indexer.setPower(0);
                }
            }
            DJIMotor::s_sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

// #include "main.h"
// #include "Infantry.h"
//// #include <jetson.h>
// #include <cstdlib>
// #include <commands/RamseteCommand.h>
// #include <iostream>
//// #include " COMPONENT_SD/include/SD/SDBlockDevice.h"
//// #include "storage/blockdevice/COMPONENT_SD/include/SD/SDBlockDevice.h"
//// #include "SDBlockDevice.h"
//// #include "storage/blockdevice/COMPONENT_SD/source/SDBlockDevice.cpp"
//
//// SDBlockDevice sd(MBED_CONF_SD_SPI_MOSI, MBED_CONF_SD_SPI_MISO, MBED_CONF_SD_SPI_CLK, MBED_CONF_SD_SPI_CS);
//
// #define PI 3.14159265
//
// #define LOWERBOUND 1000
// #define UPPERBOUND 2000
//
// I2C i2c(I2C_SDA, I2C_SCL);
// Chassis chassis(1, 2, 3, 4, &i2c);
// RamseteCommand command(
//        Pose2D(0, 0, 0), Pose2D(20, 20, 0), 2, &chassis);
// DigitalOut led(L26);
// DigitalOut led2(L27);
//
// DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
// DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
// DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left
//
// int pitchval = 0;
//
// DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
// int indexJamTime = 0;
// int lastJam = 0;
//
// DJIMotor smm_switcher(4, CANHandler::CANBUS_2, C610); // TODO: need to implement this
//
// unsigned long cT = 0;
// unsigned long forwardTime = 250;
// unsigned long reverseTime = 300;
// unsigned long totalTime;
//
// DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
// DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
// DJIMotor flyWheelMotors[] = {RFLYWHEEL, LFLYWHEEL};
// DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap
//
// void setFlyWheelSpeed(int speed)
//{
//    LFLYWHEEL.setSpeed(-speed);
//    RFLYWHEEL.setSpeed(speed);
//}
//
// Thread imuThread;
//
//// double getPitchAngle(geometry_msgs__msg__Vector3Stamped jetsonAngles) {
////     return asin(-jetsonAngles.vector.y);
//// }
////
//// double getYawAngle(geometry_msgs__msg__Vector3Stamped jetsonAngles) {
////     return atan2(jetsonAngles.vector.x, jetsonAngles.vector.z);
//// }
//
// void runImuThread()
//{
//    chassis.initializeImu();
//    while (true)
//    {
//        chassis.readImu();
//        ThisThread::sleep_for(25);
//    }
//}
//
// void pitchSetPosition(){
//
//    int pitchSetPoint = (-rY / 1.25) + 6000;
//
//    /* TODO: test min and max pitch position */
//
//    // if(pitchSetPoint > 9000)
//    //     pitchSetPoint = 9000;
//
//    // else if(pitchSetPoint < 5000)
//    //     pitchSetPoint = 5000;
//
//    pitch.setPosition(pitchSetPoint);
//    pitch2.setPower(-pitch.powerOut);
//}
//
// int main()
//{
//
//    imuThread.start(runImuThread);
//    float speedmultiplier = 3;
//    float powmultiplier = 2;
//    float translationalmultiplier = 1.5; // was 3
//    float beybladespeedmult = 1;
//
//    DJIMotor::setCANHandlers(&canHandler1, &canHandler2, false, false);
//
//    //    Jetson::init();
//
//    // LB.setSpeedPID(1.75, 0.351, 5.63);
//    // RF.setSpeedPID(1.073, 0.556, 0);
//    // RB.setSpeedPID(1.081, 0.247, 0.386);
//    // LF.setSpeedPID(.743, 0.204, 0.284);
//    pitch.setPositionPID(35, 0.11, 0.42);
//    //    pitch.setPositionPID(0, 0, 0);
//    pitch.setPositionIntegralCap(10000);
//    pitch.setPositionOutputCap(15000);
//    pitch2.setPositionOutputCap(15000);
//    LFLYWHEEL.setSpeedPID(1, 0, 0);
//    RFLYWHEEL.setSpeedPID(1, 0, 0);
//
//    pitch.useAbsEncoder = true;
//    pitch.justPosError = false;
//
//    yaw.setPositionPID(4.6, 0, 0.5);
//    yaw.setPositionIntegralCap(10000);
//    yaw.useAbsEncoder = false;
//    yaw.justPosError = true;
//
//    gearSwap.setPositionPID(4.0, 0, 0.5);
//    gearSwap.setPositionIntegralCap(10000);
//
//    indexer.setSpeedPID(1.94, 0.002, 0.166);
//    indexer.setSpeedIntegralCap(500000);
//
//    chassis.setBrakeMode(Chassis::COAST);
//
//    command.initialize();
//
//    unsigned long loopTimer = us_ticker_read() / 1000;
//
//    int indexJamTime = 0;
//
//    bool strawberryJam = false;
//    int refLoop = 0;
//
//    int yawSetPoint = 0;
//    double rotationalPower = 0;
//
//    DJIMotor::getFeedback();
//    double beybladeSpeed = 2;
//    bool beybladeIncreasing = true;
//
//    float chassis_power;
//    uint16_t chassis_power_limit;
//
//    int counter = 0;
//
//    unsigned long lastTime = 0;
//
//    while (true){
//        unsigned long timeStart = us_ticker_read() / 1000;
//
//        if (timeStart - loopTimer > 25){
//            led = !led;
//            loopTimer = timeStart;
//            remoteRead();
//            chassis.periodic();
//
//            refLoop++;
//            if (refLoop >= 5)
//            {
//                refereeThread(&referee);
//                refLoop = 0;
//                led2 = !led2;
//            }
//            if (!chassis.allMotorsConnected()){
//                chassis.driveXYR({0, 0, 0});
//
//            }else if (rS == Remote::SwitchState::DOWN){          // All non-serializer motors activated
//                int LFa = lY + lX * translationalmultiplier + rX;
//                int RFa = lY - lX * translationalmultiplier - rX;
//                int LBa = lY - lX * translationalmultiplier + rX;
//                int RBa = lY + lX * translationalmultiplier - rX;
//
//                chassis_power = ext_power_heat_data.data.chassis_power;
//                unsigned long time = us_ticker_read() / 1000;
//                chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;
//                chassis.driveTurretRelativePower(chassis_power, chassis_power_limit, {lX * 5.0, lY * 5.0, 0}, yaw.getData(MULTITURNANGLE) * 360.0 / 8192, int(time - timeStart) * 1000, rotationalPower);
//
//                lastTime = time;
//
//                //
//                //                    printf("ref: %f\n", ref_chassis_power);
//
//                //                    double setpoint = getPitchAngle(Jetson::cv) * 4096 / PI + 6715;
//                //                    printf("Setpoint: %i\n", (int) setpoint);
//                //                    pitch.setPosition(setpoint);
//
//                pitchSetPosition();
//
//                //                 yaw.setSpeed(rX/100);
//
//                //                yawSetpoint = (4096 * getYawAngle(Jetson::cv) / PI - yaw.getData(ANGLE));
//                //                yawSetpoint = 0;
//                //                yawSetpoint -= rX / 10.0;
//                //                yawSetpoint -= rX / 10.0;
//                //                    if (!command.isFinished()) {
//                //                        printf("running command!\n");
//                //                        command.execute();
//                //                    }
//
//                //                } else {
//                //                    chassis.driveXYR(0, 0, 0);
//                //                        printf("ESTIMATE VS TIME:\n");
//                //                        for (auto & i : chassis.testData) {
//                //                            printf("%i\t%i\n", i[0], i[1]);
//                //                        }
//                //                        printf("\n\n\nMULTITURN VS TIME:\n");
//                //                        for (auto & i : chassis.testData) {
//                //                            printf("%i\t%i\n", i[0], i[2]);
//                //                        }
//                //                        printf("\n\n\nVELOCITY VS TIME::\n");
//                //                        for (auto & i : chassis.testData) {
//                //                            printf("%i\t%i\n", i[0], i[3]);
//                //                        }
//                //                }
//                //                printf("Angle: %i\n", (int) pitch.getData(ANGLE));
//                //                pitch.setPosition((rY * 0.9) + 6400);
//                printf("Pitch angle: %i ", (int) pitch.getData(ANGLE));
//                printf("Pitch power: %i ", (int) pitch.powerOut);
//                printf("Pitch2 angle: %i ", (int) pitch2.getData(ANGLE));
//                printf("Pitch2 power: %i\n", (int) pitch2.powerOut);
//                //                printf("Yaw motor angle: %i\n", (int) yaw.getData(MULTITURNANGLE));
//                //                printf("Yaw power: %i\n", (int) yawgetData(MULTI.powerOut);
//                //                printf("Flywheel out: %i\n", (int) LFLYWHEEL.powerOut);
//                //                printf("Flywheel speed: %i\n", (int) LFLYWHEEL.getData(VELOCITY));
//                //                printf("Flywheel pos: %i\n", (int) LFLYWHEEL.getData(ANGLE));
//                //                pitch.setPower((int) (rY * 3));
//                //                printf("Setting power: %i\n", (int) pitch.powerOut);
//                // yaw.setSpeed(rX/100);
//                yawSetPoint += rX / 12;
//                //                yawSetpoint =  - 3 * rX;
//                yaw.setPosition(-chassis.getHeadingDegrees() * 8192 / 360 + yaw.getData(MULTITURNANGLE) - yawSetPoint);
//                //                yaw.setPosition(0);
//            }
//            else if (rS == Remote::SwitchState::MID || rS == Remote::SwitchState::UNKNOWN)
//            { // disable all the non-serializer components
//                //                chassis.driveXYR(0,0,0);
//                chassis.driveFieldRelative({0, 0, 0});
//                yaw.setPower(0);
//                pitch.setPower(0);
//                pitch2.setPower(0);
//            }
//            else if (rS == Remote::SwitchState::UP)
//            { // beyblade mode
//                double ref_chassis_power = ext_power_heat_data.data.chassis_power;
//                // printf("Ref power: %i\n", (int) ref_chassis_power);
//                unsigned long time = us_ticker_read() / 1000;
//                int max_power = ext_game_robot_state.data.chassis_power_limit;
//                chassis.driveXYRPower(ref_chassis_power, max_power, 5 * lX, 5 * lY, time - lastTime, true, rotationalPower);
//                lastTime = time;
//                // chassis.beyblade(lX * 5.0, lY * 5.0, yaw.getData(MULTITURNANGLE) * 360.0 / 8192 + 90, false);
//                pitchSetPosition();
//                yawSetPoint += rX / 12;
//                //                yawSetpoint =  - 3 * rX;
//                yaw.setPosition(-chassis.getHeadingDegrees() * 8192 / 360 + yaw.getData(MULTITURNANGLE) - yawSetPoint);
//                //                yaw.setPosition(0);
//                //                yaw.setPower(0); pitch.setPower(0);
//            }
//
//
//            // Referee barrel temperature for both
//            uint16_t ref_chassis_temp1 = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
//            uint16_t ref_chassis_temp2 = ext_power_heat_data.data.shooter_id2_17mm_cooling_heat;
//            // std::cout << ref_chassis_temp1 << endl;
//            // Stop shooting based on Infantry's level
//            int heatMax1 = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit;
//            int heatMax2 = ext_game_robot_state.data.shooter_id2_17mm_cooling_limit;
//
//            if (lS == Remote::SwitchState::UP)
//            {
//                gearSwap.setPower(1000);
//                printf("Ref temp barrel 1: %i Ref temp barrel 2: %i\n", (int)ref_chassis_temp1, (int) ref_chassis_temp2);
//
//                printf("MMAX barrel temp currently # 1: %i MAX barrel temp currently # 2: %i\n", (int)heatMax1, (int)heatMax2);
//
//                // cooling rate
//                int coolrate = ext_game_robot_state.data.shooter_id1_17mm_cooling_rate;
//
//                if (us_ticker_read() - timeStart > 1)
//                    ref_chassis_temp1 = ref_chassis_temp1 - coolrate / 10;
//
//                if (ref_chassis_temp1 < heatMax1) // not finish yet
//                {
//                    // indexer.setPower(1200);
//                    indexer.setSpeed(2000);
//                    // printf("Indexer data: %i %i %i\n", (int) (indexer.powerOut), (int) (indexer.getData(ANGLE)), (int) indexer.getData(VELOCITY));
//                    setFlyWheelSpeed(20000);
//                }
//                else
//                {
//                    indexer.setSpeed(0);
//                    setFlyWheelSpeed(0);
//                }
//
//                //               if (timeStart / 100 == 0) {
//                //                   printf("Angle: %i\n", (int) 0);
//                //                    chassis.printMotorAngle();
//                //               }
//            }
//            else if (lS == Remote::SwitchState::MID)
//            { // disable serializer
//                gearSwap.setPower(0);
//                indexer.setPower(0);
//                setFlyWheelSpeed(0);
//            }
//            else if (lS == Remote::SwitchState::DOWN)
//            {
//                gearSwap.setPower(-1100);
//                setFlyWheelSpeed(20000); // was 0
//
//                // printf("TORQ:%d VEL:%d\n",indexer.getData(TORQUE), indexer.getData(VELOCITY));
//                if (abs(indexer.getData(TORQUE)) > 100 & abs(indexer.getData(VELOCITY)) < 20)
//                { // jam
//                    indexJamTime = us_ticker_read() / 1000;
//                }
//                if (us_ticker_read() / 1000 - indexJamTime < 1000)
//                {
//                    indexer.setPower(-300); // jam
//                    // printf("JAMMMMM- ");
//                }
//                else if (us_ticker_read() / 1000 - indexJamTime < 1500)
//                {
//                    //                    indexer.setPower(3000); //jam
//                    indexer.setPower(0);
//                    // printf("POWER FORWARD- ");
//                }
//                else
//                {
//                    // indexer.setPower(-900);
//                    indexer.setSpeed(1000); // was 650
//                }
//            }
//            DJIMotor::sendValues();
//        }
//        unsigned long timeEnd = us_ticker_read() / 1000;
//        DJIMotor::getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}