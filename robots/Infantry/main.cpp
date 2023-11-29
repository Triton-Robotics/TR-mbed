#include "main.h"
#include "Infantry.h"
// #include <jetson.h>
#include <cstdlib>
#include <commands/RamseteCommand.h>
#include <iostream>
// #include " COMPONENT_SD/include/SD/SDBlockDevice.h"
// #include "storage/blockdevice/COMPONENT_SD/include/SD/SDBlockDevice.h"
// #include "SDBlockDevice.h"
// #include "storage/blockdevice/COMPONENT_SD/source/SDBlockDevice.cpp"

// SDBlockDevice sd(MBED_CONF_SD_SPI_MOSI, MBED_CONF_SD_SPI_MISO, MBED_CONF_SD_SPI_CLK, MBED_CONF_SD_SPI_CS);

#define PI 3.14159265

#define LOWERBOUND 1000
#define UPPERBOUND 2000

I2C i2c(I2C_SDA, I2C_SCL);
Chassis chassis(1, 2, 3, 4, &i2c);
DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left
DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);
DJIMotor gearSwap(4, CANHandler::CANBUS_2, M2006); // gear swap

RamseteCommand command(
        Pose2D(0, 0, 0), Pose2D(20, 20, 0), 2, &chassis);
DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

void setFlyWheelSpeed(int speed)
{
    LFLYWHEEL.setSpeed(-speed);
    RFLYWHEEL.setSpeed(speed);
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

void pitchSetPosition(){

    int pitchSetPoint = (-remote.rightY() * 0.5) + 6200;

    /* TODO: test min and max pitch position */

    // if(pitchSetPoint > 9000)
    //     pitchSetPoint = 9000;

    // else if(pitchSetPoint < 5000)
    //     pitchSetPoint = 5000;

    pitch.setPosition(pitchSetPoint);
    //pitch2.setPower(-pitch.powerOut);
    //printf("%d.%d %d\n", pitchSetPoint, pitch.getData(ANGLE), pitch.powerOut);

}

/*
 * shoot variable set to true when leftSwitch turned up
 * indexer positionn conntrol to move rotate ~3 ball positions
 * indexer movement controlled only here, switch states used to set conditions for this part
 * need to make all variables used global: shootTargetPosition, timeSure, prevTimeSure
 */
// void burstFire(shoot){
//     if (shoot){
//         // shootPosition = indexer>>MULTITURNANGLE;
//         // if (shootPosition >= shootTargetPosition){

//         if (indexer>>MULTITURNANGLE >= shootTargetPosition){
//             indexer.setSpeed(0); //before -1000
//             shoot = false;
//         } else {
//             timeSure = us_ticker_read();
//             indexer.setSpeed(sure.calculate(shootTargetPosition, indexer>>MULTITURNANGLE, timeSure - prevTimeSure));
//             prevTimeSure = timeSure;
//         }
//     } else {
//         indexer.setSpeed(0);
//     }
// }


int main(){

    imuThread.start(runImuThread);
    float speedmultiplier = 3;
    float powmultiplier = 2;
    float translationalmultiplier = 1.5; // was 3
    float beybladespeedmult = 1;

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);

    pitch.setPositionPID(17.3, 0.03, 8.5); //12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
    pitch.setPositionIntegralCap(60000);
    pitch.setPositionOutputCap(100000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;

    LFLYWHEEL.setSpeedPID(7.5, 0, 0.04);
    RFLYWHEEL.setSpeedPID(7.5, 0, 0.04);

    // yaw.setPositionPID(10, 0, 5.5);
    // yaw.setPositionIntegralCap(10000);
    yaw.setPositionPID(10.5, 0.2, 4.4);
    yaw.setPositionIntegralCap(10000);
    yaw.useAbsEncoder = false;

    PID yawIMU(200.0,0.1,150,20000,8000);//7.0,0.02,15.0,20000,8000

    gearSwap.setPositionPID(4.0, 0, 0.5);
    gearSwap.setPositionIntegralCap(10000);

    //indexer.setSpeedPID(3.1, 0.07, 4);
    indexer.setSpeedPID(1, 0, 1);
    indexer.setSpeedIntegralCap(8000);

    chassis.setBrakeMode(Chassis::COAST);

    command.initialize();

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

    bool shoot = false;
    //int shootPosition;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;
    PID sure(0.5,0,1);
    sure.setOutputCap(4000);
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
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2= !led2;
            }

            chassis_power = ext_power_heat_data.data.chassis_power;
            chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;

            ref_chassis_temp1 = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
            ref_chassis_temp2 = ext_power_heat_data.data.shooter_id2_17mm_cooling_heat;

            heatMax1 = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit;
            heatMax2 = ext_game_robot_state.data.shooter_id2_17mm_cooling_limit;

            ref_yaw = int(ext_game_robot_pos.data.yaw);

            //printff("%d\n",chassis.getHeadingDegrees());
            //printf("%d %d\n", lX, rX);

            if (remote.rightSwitch() == Remote::SwitchState::UP){          // All non-serializer motors activated
                led3 = 1;
                int LFa = remote.leftY() + remote.leftX() * translationalmultiplier + remote.rightX();
                int RFa = remote.leftY() - remote.leftX() * translationalmultiplier - remote.rightX();
                int LBa = remote.leftY() - remote.leftX() * translationalmultiplier + remote.rightX();
                int RBa = remote.leftY() + remote.leftX() * translationalmultiplier - remote.rightX();

                unsigned long time = us_ticker_read();
                chassis.driveTurretRelativePower(chassis_power, chassis_power_limit, {remote.leftX() * 5.0, remote.leftY() * 5.0, 0}, yaw.getData(ANGLE) * 360.0 / 8192 + 180, int(time - lastTime), rotationalPower);

                lastTime = time;
                pitchSetPosition();

                //yawSetPoint += rX / 36;
                //yaw.setPosition(-chassis.getHeadingDegrees() * 8192 / 360 + yaw.getData(MULTITURNANGLE) - yawSetPoint);

                yaw.setPower(-remote.rightX() * 10);

                //printff("Yaw:%d Pitch%d\n",(int)((double)(yaw>>MULTITURNANGLE) * 360 / 8192),(int)((double)((pitch>>MULTITURNANGLE) - 6000) * 360 / 8192));
                //printff("y:%d p:%d\n",yaw>>MULTI,pitch>>ANGLE);
            }
            else if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN){ // disable all the non-serializer components
                led3 = 0;
                chassis.driveFieldRelative({0, 0, 0});
                yaw.setPower(0);
                pitch.setPower(0);
                pitch2.setPower(0);
                yawSetPoint = int(ext_game_robot_pos.data.yaw);
                //printff("whe%d\n",Wh);
                pitchSetPosition();

            }else if (remote.rightSwitch() == Remote::SwitchState::DOWN){           // beyblade mode
                led3 = 1;
                unsigned long time = us_ticker_read();
                double r = 4000;
                chassis.driveXYRPower(chassis_power, chassis_power_limit, 5 * remote.leftX(), 5 * remote.leftY(), int(time - lastTime), true, r);
                lastTime = time;
                pitchSetPosition();
                yawSetPoint += remote.rightX() / 110;
                yawSetPoint %= 360;
                while(yawSetPoint < 0)
                    yawSetPoint += 360;
                //yaw.setPosition(-ref_yaw * 8192 / 360 + yaw.getData(MULTITURNANGLE) - yawSetPoint);
                yaw.setPower(yawIMU.calculatePeriodic(float(calculateDeltaYaw(ref_yaw, yawSetPoint)), us_ticker_read() - yawTime));
                printff("%dya imu:[%d] pwr%d\n",yawSetPoint,ref_yaw, yaw>>POWEROUT);
            }
            yawTime = us_ticker_read();


            if (shoot){
                if (indexer>>MULTITURNANGLE >= shootTargetPosition){
                    indexer.setSpeed(0); //before -1000
                    shoot = false;
                } else {
                    timeSure = us_ticker_read();
                    indexer.setSpeed(sure.calculate(shootTargetPosition, indexer>>MULTITURNANGLE, timeSure - prevTimeSure));
                    prevTimeSure = timeSure;
                }
            } else {
                indexer.setSpeed(0);
            }

            //burstFire(shoot);


            if (remote.leftSwitch() == Remote::SwitchState::MID){
                //gearSwap.setPower(-1500);
                double mps = ext_game_robot_state.data.shooter_id1_17mm_speed_limit * 0.9;
                double rpm = 60 * (mps / 0.03) / (3.14159 * 2);
                //setFlyWheelSpeed(rpm);
                RFLYWHEEL.setSpeed(7000);
                LFLYWHEEL.setSpeed(-7000);
                //indexer.setPower(0);
                shootReady = true;
                //burstTimestamp = us_ticker_read();

            }else if (remote.leftSwitch() == Remote::SwitchState::DOWN || remote.leftSwitch() == Remote::SwitchState::UNKNOWN){          // disable serializer
                //gearSwap.setPower(-1500);
                //indexer.setPower(0);
                setFlyWheelSpeed(0);


                shootReady = true;

            }else if (remote.leftSwitch() == Remote::SwitchState::UP){
                //gearSwap.setPower(-1500);
                double mps = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;
                double rpm = 60 * (mps / 0.03) / (3.14159 * 2);
                //setFlyWheelSpeed(rpm); // was 0 * 0/
                RFLYWHEEL.setSpeed(7000);
                LFLYWHEEL.setSpeed(-7000);

                // burstTimestamp used for previous indexer control for burst fire?
                // if((us_ticker_read() - burstTimestamp)/1000 < 99){
                //     indexer.setPower(8000);
                // }else{
                //     indexer.setPower(0);
                // }
                if (shootReady){
                    shootReady = false;
                    shoot = true;
                    shootTargetPosition = 8192 * 12 + (indexer>>MULTITURNANGLE);
                }



                //printff("%d\n", indexer.powerOut);
                printff("v:%d\n", indexer.getData(VELOCITY));
                if(heatMax1 - ref_chassis_temp1 < 60){
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