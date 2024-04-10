//#include "main.h"
////#include "Infantry.h"
//#include <cstdlib>
//#include <commands/RamseteCommand.h>
//#include <iostream>
//#define PI 3.14159265
//#define LOWERBOUND 1000
//#define UPPERBOUND 2000
//I2C i2c(I2C_SDA, I2C_SCL);
//Chassis chassis(1, 2, 3, 4, &i2c);
//DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
//DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
//DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left, not functioning
//DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
//DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
//DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
//DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap
//RamseteCommand command(
//        Pose2D(0, 0, 0), Pose2D(20, 20, 0), 2, &chassis);
//DigitalOut led(L26);
//DigitalOut led2(L27);
//DigitalOut led3(L25);
//
//int calculateDeltaYaw(int ref_yaw, int beforeBeybladeYaw){
//    int deltaYaw = beforeBeybladeYaw - ref_yaw;
//
//    if(abs(deltaYaw) > 180){
//        if(deltaYaw > 0)
//            deltaYaw -= 360;
//        else
//            deltaYaw += 360;
//    }
//    return deltaYaw;
//}
//
//Thread imuThread;
//
//void runImuThread()
//{
//    chassis.initializeImu();
//    while (true)
//    {
//        chassis.readImu();
//        ThisThread::sleep_for(25);
//    }
//}
//
//void pitchSetPosition(){
//    //int pitchSetPoint = (-remote.rightY() * 0.5) + 6200;
//    int pitchSetPoint = 5960 - (5960 - 5350)/660.0 * remote.rightY();
//
//    // low
//    if(pitchSetPoint > 6570)
//        pitchSetPoint = 6570;
//        // high
//    else if(pitchSetPoint < 5350)
//        pitchSetPoint = 5350;
//
//    pitch.setPosition(pitchSetPoint);
//    //printff("%d.%d %d\n", pitchSetPoint, pitch.getData(ANGLE), pitch.powerOut);
//}
//
//
//
//int main(){
//
//    imuThread.start(runImuThread);
//    float speedmultiplier = 3;
//    float powmultiplier = 2;
//    float translationalmultiplier = 1.5; // was 3
//    float beybladespeedmult = 1;
//    PID yawIMU(200.0,0.1,150,20000,8000);//7.0,0.02,15.0,20000,8000
//
//    chassis.setBrakeMode(Chassis::COAST);
//
//    command.initialize();
//
//    unsigned long loopTimer = us_ticker_read();
//
//    int refLoop = 0;
//    int ref_yaw;
//
//    double rotationalPower = 0;
//
//    DJIMotor::s_getFeedback();
//    double beybladeSpeed = 2;
//    bool beybladeIncreasing = true;
//
//    float chassis_power;
//    uint16_t chassis_power_limit;
//    uint16_t ref_chassis_temp1;
//    uint16_t ref_chassis_temp2;
//    uint16_t heatMax1;
//    uint16_t heatMax2;
//
//    int counter = 0;
//
//    unsigned long lastTime = 0;
//    unsigned long yawTime = us_ticker_read();
//    //unsigned long burstTimestamp = 0;
//
//
//    while (true){
//        unsigned long timeStart = us_ticker_read();
//
//        if ((timeStart - loopTimer) / 1000 > 25){
//            gearSwap.setPower(2000);
//            led = !led;
//            loopTimer = timeStart;
//            remoteRead();
//            chassis.periodic();
//
//            refLoop++;
//            if (refLoop >= 5){
//                refereeThread(&referee);
//                refLoop = 0;
//                led2= !led2;
//            }
//
//
//            /**
//             * Variables
//             */
//            chassis_power = ext_power_heat_data.data.chassis_power;
//            chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;
//
//            ref_chassis_temp1 = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
//            ref_chassis_temp2 = ext_power_heat_data.data.shooter_id2_17mm_cooling_heat;
//
//            heatMax1 = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit;
//            heatMax2 = ext_game_robot_state.data.shooter_id2_17mm_cooling_limit;
//
//            ref_yaw = int(ext_game_robot_pos.data.yaw);
//
//            //printff("%d\n",chassis.getHeadingDegrees());
//            //printf("%d %d\n", lX, rX);
//
//
//            /**
//             * rightSwitch controls
//             */
//            if (remote.rightSwitch() == Remote::SwitchState::UP){          // All non-serializer motors activated
//                led3 = 1;
//                unsigned long time = us_ticker_read();
//                chassis.driveTurretRelativePower(chassis_power, chassis_power_limit, {remote.leftX() * 5.0, remote.leftY() * 5.0, 0}, yaw.getData(ANGLE) * 360.0 / 8192 + 180, int(time - lastTime), rotationalPower);
//                lastTime = time;
//            } else if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN){ // disable all the non-serializer components
//                chassis.driveFieldRelative({0, 0, 0});
//            } else if (remote.rightSwitch() == Remote::SwitchState::DOWN){           // beyblade mode
//                unsigned long time = us_ticker_read(); //time for pid
//                double r = 4000;
//                chassis.driveXYRPower(chassis_power, chassis_power_limit, 5 * remote.leftX(), 5 * remote.leftY(), int(time - lastTime), true, r);
//                lastTime = time;
//            }
//
//
//            DJIMotor::s_sendValues();
//        }
//        unsigned long timeEnd = us_ticker_read() / 1000;
//        DJIMotor::s_getFeedback();
//        ThisThread::sleep_for(1ms);
//    }
//}




#include "main.h"
// #include "Infantry.h"
// #include <jetson.h>
#include <cstdlib>
#include <commands/RamseteCommand.h>
#include <iostream>

#define PI 3.14159265

#define LOWERBOUND 1000
#define UPPERBOUND 2000

I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);
DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left, not functiong
DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap

RamseteCommand command(
Pose2D(0, 0, 0), Pose2D(20, 20, 0), 2, &chassis);
DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

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

int main(){

    imuThread.start(runImuThread);
    float speedmultiplier = 3;
    float powmultiplier = 2;
    float translationalmultiplier = 1.5; // was 3
    float beybladespeedmult = 1;

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);


    //     yaw.setPositionPID(10.5, 0.2, 4.4);
    //     yaw.setPositionIntegralCap(10000);
    yaw.useAbsEncoder = false;

    PID yawIMU(200.0,0.1,150,20000,8000);//7.0,0.02,15.0,20000,8000
    //     PID yawBeyblade(30.0,0,200);
    PID yawBeyblade(1, 0, 200, 6000, 220);

    yaw.setSpeedPID(500, 0, 1000);

    //     command.initialize();

    unsigned long loopTimer = us_ticker_read();

    int refLoop = 0;
    int ref_yaw;

    //     int yawSetPoint = int(-ext_game_robot_pos.data.yaw * 8192 / 360);
    chassis.readImu();
    int yawSetPoint = chassis.getHeadingDegreesYaw()+ 180;
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
    //unsigned long burstTimestamp = 0;
    double yawDesiredImu;

    unsigned long timeSure;
    unsigned long prevTimeSure;

    while (true){
        unsigned long timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            gearSwap.setPower(2000);
            led = !led;
            loopTimer = timeStart;
            remoteRead();
            chassis.periodic();

            refLoop++;
            if (refLoop >= 25){
                refereeThread(&referee);
                refLoop = 0;
                led2= !led2;
                //                 led3 = !led3;
                //printff("Angle yaw: %d; Set point: %d; Power out: %d %\n", chassis.getHeadingDegreesYaw(), yawSetPoint, yaw>>POWEROUT);
                //                 printff("Yaw: %f, %f, %f %d %d\n", yaw.pidSpeed.pC, yaw.pidSpeed.iC, yaw.pidSpeed.dC, yaw>>VELOCITY, yaw>>VALUE);

                }


                /**
                 * Variables
                 */
                chassis_power = ext_power_heat_data.data.chassis_power;
                chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;

                ref_chassis_temp1 = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
                ref_chassis_temp2 = ext_power_heat_data.data.shooter_id2_17mm_cooling_heat;

                heatMax1 = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit;
                heatMax2 = ext_game_robot_state.data.shooter_id2_17mm_cooling_limit;

                ref_yaw = int(ext_game_robot_pos.data.yaw);

                /**
                 * rightSwitch controls
                 */
            if (remote.rightSwitch() == Remote::SwitchState::UP){          // All non-serializer motors activated
                unsigned long time = us_ticker_read();
                lastTime = time;
                yaw.setSpeed(-remote.rightX() /11.0 );

                //                 yaw.setPower(-remote.rightX() * 10);
            } else if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN){ // disable all the non-serializer components
                yaw.setPower(0);
            } else if (remote.rightSwitch() == Remote::SwitchState::DOWN){           // beyblade mode
                //                 yawDesiredImu  -=  remote.rightX() * 5;
                yawSetPoint -= remote.rightX() / 100;
                yawSetPoint = (yawSetPoint+360) % 360;
                timeSure = us_ticker_read();
                //                 Determine which direction yaw should turn
                //                 if ((yawSetPoint + 720 -  chassis.getHeadingDegreesYaw() - 180) % 360 < (yawSetPoint + 180 - chassis.getHeadingDegreesYaw())%360){
                //                     //
                //                 } else {
                //
                //                 }
                yaw.setSpeed(yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint, chassis.getHeadingDegreesYaw()+180, 360), timeSure - prevTimeSure)); //
                prevTimeSure = timeSure;

                //                 unsigned long time = us_ticker_read(); //time for pid
                //                 double r = 4000;
                //                 lastTime = time;
                //                 yawSetPoint += remote.rightX() / 110;
                //                 yawSetPoint %= 360;
                //                 while(yawSetPoint < 0)
                //                     yawSetPoint += 360;
                //                 yaw.setPower(yawIMU.calculatePeriodic(float(calculateDeltaYaw(ref_yaw, yawSetPoint)), us_ticker_read() - yawTime)); //
                //printff("%dya imu:[%d] pwr%d\n",yawSetPoint,ref_yaw, yaw>>POWEROUT);
            }
            yawTime = us_ticker_read();
            DJIMotor::s_sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}