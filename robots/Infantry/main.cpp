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
DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left, not functioning
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
    //int pitchSetPoint = (-remote.rightY() * 0.5) + 6200;
    //1340 2900

    int pitchSetPoint = 2120 - (2120 - 1340)/660.0 * remote.rightY();

    // low
    if(pitchSetPoint > 2900)
        pitchSetPoint = 2900;
        // high
    else if(pitchSetPoint < 1340)
        pitchSetPoint = 1340;


//    int pitchSetPoint = 5960 - (5960 - 5350)/660.0 * remote.rightY();
//
//    // low
//    if(pitchSetPoint > 6570)
//        pitchSetPoint = 6570;
//        // high
//    else if(pitchSetPoint < 5350)
//        pitchSetPoint = 5350;

    pitch.setPosition(pitchSetPoint);
    //printff("%d.%d %d\n", pitchSetPoint, pitch.getData(ANGLE), pitch.powerOut);
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

    pitch.setPositionPID(18, 0.01, 850); //12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
    pitch.setPositionIntegralCap(6000);
    //pitch.setPositionOutputCap(100000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;

    /* Pitch2 not functioning, only using pitch defined above ^
    */
    // pitch2.setPositionPID(17.3, 0.03, 8.5); //12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
    // pitch2.setPositionIntegralCap(60000);
    // pitch2.setPositionOutputCap(100000);
    // pitch2.pidPosition.feedForward = 0;
    // pitch2.outputCap = 32760;
    // pitch2.useAbsEncoder = true;

    LFLYWHEEL.setSpeedPID(7.5, 0, 0.04);
    RFLYWHEEL.setSpeedPID(7.5, 0, 0.04);

    yaw.setPositionPID(10.5, 0.2, 4.4);
    yaw.setPositionIntegralCap(10000);
    yaw.useAbsEncoder = false;

    PID yawIMU(200.0,0.1,150,20000,8000);//7.0,0.02,15.0,20000,8000

    gearSwap.setPositionPID(4.0, 0, 0.5);
    gearSwap.setPositionIntegralCap(10000);

    indexer.setSpeedPID(1, 0, 1);
    indexer.setSpeedIntegralCap(8000);

    chassis.setBrakeMode(Chassis::COAST);

    command.initialize();

    unsigned long loopTimer = us_ticker_read();

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
    //unsigned long burstTimestamp = 0;

    //Variables & PID for burst fire
    bool shoot = false;
    //int shootPosition;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;
    //PID for indexer angle position control. Surely there are better names then "sure"...
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

            //printff("%d\n",chassis.getHeadingDegrees());
            //printf("%d %d\n", lX, rX);


            /**
             * RightSwitch controls: Pitch, Yaw, Chassis
             * Up: Pitch enabled, yaw and chassis seperate
             * Mid or Unkown: Off. All power set to 0
             * Down: Pitch enabled, yaw and chassis Beyblade
             */
            if (remote.rightSwitch() == Remote::SwitchState::UP){          // All non-serializer motors activated
                led3 = 1;
                unsigned long time = us_ticker_read();
                chassis.driveTurretRelativePower(chassis_power, chassis_power_limit, {remote.leftX() * 5.0, remote.leftY() * 5.0, 0}, yaw.getData(ANGLE) * 360.0 / 8192 + 180, int(time - lastTime), rotationalPower);
                lastTime = time;
//                pitchSetPosition();
                yaw.setPower(-remote.rightX() * 10);
            } else if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN){ // disable all the non-serializer components
                chassis.driveFieldRelative({0, 0, 0});
                yaw.setPower(0);
                pitch.setPower(0);
            } else if (remote.rightSwitch() == Remote::SwitchState::DOWN){           // beyblade mode
                unsigned long time = us_ticker_read(); //time for pid
                double r = 4000;
                chassis.driveXYRPower(chassis_power, chassis_power_limit, 5 * remote.leftX(), 5 * remote.leftY(), int(time - lastTime), true, r);
                lastTime = time;
//                pitchSetPosition();
                yawSetPoint += remote.rightX() / 110;
                yawSetPoint %= 360;
                while(yawSetPoint < 0)
                    yawSetPoint += 360;
                yaw.setPower(yawIMU.calculatePeriodic(float(calculateDeltaYaw(ref_yaw, yawSetPoint)), us_ticker_read() - yawTime)); //
                //printff("%dya imu:[%d] pwr%d\n",yawSetPoint,ref_yaw, yaw>>POWEROUT);
            }
            yawTime = us_ticker_read();


            // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
            // only shoot when left switch changes from down/unknown/mid to up
            // if left switch remains at up state, indexer stops after 3-5 balls
            if (shoot){
                if (indexer>>MULTITURNANGLE >= shootTargetPosition){
                    indexer.setSpeed(0);
                    shoot = false;
                } else {
                    timeSure = us_ticker_read();
                    indexer.setSpeed(sure.calculate(shootTargetPosition, indexer>>MULTITURNANGLE, timeSure - prevTimeSure)); //
                    prevTimeSure = timeSure;
                }
            } else {
                indexer.setSpeed(0);
            }


            /**
             * left switch controls burst fire: indexer and flywheels
             * state down or unknown: no motion
             * state mid: flywheels running, resetting shoot state
             * state up: flywheels continues running, turn indexer if state set by state mid
             */
            if (remote.leftSwitch() == Remote::SwitchState::UP ){
                // Monitors state of left switch at previous loop and determine whether to turn indexer on
                // if left switch was at other states, turn indexer on
                // otherwise, continue the burstfire and stop after 3-5 shots
                // $shootReady local to if block, $shoot variable used above
                if (shootReady){
                    shootReady = false;
                    shoot = true;
                    shootTargetPosition = 8192 * 12 + (indexer>>MULTITURNANGLE);
                }
            } else {
                //SwitchState state set to mid/down/unknown
                shootReady = true;
            }
            if (remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN){
//                double mps = ext_game_robot_state.data.shooter_id1_17mm_speed_limit * 0.9;
//                double rpm = 60 * (mps / 0.03) / (3.14159 * 2);
                RFLYWHEEL.setSpeed(7000);
                LFLYWHEEL.setSpeed(-7000);
            } else{
                // left SwitchState set to up/mid/unknown
                RFLYWHEEL.setSpeed(0);
                LFLYWHEEL.setSpeed(0);
            }


            DJIMotor::s_sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}