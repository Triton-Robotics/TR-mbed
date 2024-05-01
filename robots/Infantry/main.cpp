#include "main.h"
#include "Infantry.h"
// #include <jetson.h>
#include <cstdlib>
//#include <commands/RamseteCommand.h>

#include <iostream>
#include "subsystems/ChassisSubsystem.h"

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
#define RUNSPIN 1.0

I2C i2c(I2C_SDA, I2C_SCL);
//Chassis chassis(1, 2, 3, 4, &i2c);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left, not functioning

DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);


// RamseteCommand command(
//     Pose2D(0, 0, 0), Pose2D(20, 20, 0), 2, &chassis);
DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

BNO055_ANGULAR_POSITION_typedef imuAngles;

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

void runImuThread()
{
    //chassis.initializeImu();
    while (true)
    {
        //chassis.readImu();
        ThisThread::sleep_for(25);
    }
}

void pitchSetPosition()
{

    int pitchSetPoint = 2120 - (2120 - 1340)/660.0 * remote.rightY();
//    merge difference
  //     int pitchSetPoint = (-remote.rightY() * 0.5) + 6200;

    /* TODO: test min and max pitch position */

    // if(pitchSetPoint > 9000)
    //     pitchSetPoint = 9000;

    // else if(pitchSetPoint < 5000)
    //     pitchSetPoint = 5000;

    // low
    if(pitchSetPoint > 2900)
        pitchSetPoint = 2900;
        // high
    else if(pitchSetPoint < 1340)
        pitchSetPoint = 1340;
    pitch.setPosition(pitchSetPoint);

    // pitch2.setPower(-pitch.powerOut);
    // printf("%d.%d %d\n", pitchSetPoint, pitch.getData(ANGLE), pitch.powerOut);
}

int main()
{

    float speedmultiplier = 3;
    float powmultiplier = 2;
    float translationalmultiplier = 1.5; // was 3
    float beybladespeedmult = 1;

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);

    pitch.setPositionPID(18, 0.01, 850); //12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
    pitch.setPositionIntegralCap(6000);
  //   merge difference:
//     pitch.setPositionPID(17.3, 0.03, 8.5); // 12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
//     pitch.setPositionIntegralCap(60000);
//    pitch.setPositionOutputCap(100000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;


    pitch.useAbsEncoder = true;
    pitch.setPositionPID(15, 0, 1700);
    pitch.setPositionOutputCap(32000);
    float currentPitch = 0;
    float desiredPitch = 0;
    float pitch_phase = 9.31 / 180.0 * PI; // 5.69 theoretical
    float InitialOffset_Ticks = 2765;
    float K = 0.75; //0.85

    LFLYWHEEL.setSpeedPID(7.5, 0, 0.04);
    RFLYWHEEL.setSpeedPID(7.5, 0, 0.04);
//     merge difference
//     PID yawIMU(200.0, 0.1, 150, 20000, 8000); // 7.0,0.02,15.0,20000,8000


    Chassis.setYawReference(&yaw, 2050); // "5604" is the number of ticks of yawOne considered to be robot-front
    Chassis.setSpeedFF_Ks(0.065);

    yaw.setSpeedPID(0.5, 0, 200);
    PID yawBeyblade(50, 0, 5);
    PID yawNonBeyblade(100, 0, 50);

    yaw.setSpeedIntegralCap(1000);
    yaw.useAbsEncoder = false;
    indexer.setSpeedPID(1, 0, 1);
    indexer.setSpeedIntegralCap(8000);
//  merge difference
//   chassis.setBrakeMode(ChassisSubsystem::BrakeMode::COAST);

    // command.initialize();

    unsigned long loopTimer = us_ticker_read();

    int refLoop = 0;
    int ref_yaw;

    int yawSetPoint = imuAngles.yaw;
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

    while (true)
    {
        unsigned long timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            led3 = !led3;

            loopTimer = timeStart;
            remoteRead();
            Chassis.periodic();

            refLoop++;
            imu.get_angular_position_quat(&imuAngles);
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;

                printff("%f %d %d %d\n", imuAngles.yaw, yawSetPoint, remote.rightX()*80, yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure));

            }





            double scalar = 1;
            double jx = remote.leftX() / 660.0 * scalar;
            double jy = remote.leftY() / 660.0 * scalar;
            double jr = remote.rightX() / 660.0 * scalar;

            double tolerance = 0.05;
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jr = (abs(jr) < tolerance) ? 0 : jr;


            currentPitch = (double(pitch.getData(ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360; // degrees

            int leftStickValue = remote.rightY();
            /**
             * RightSwitch controls: Pitch, Yaw, Chassis
             * Up: Pitch enabled, yaw and chassis seperate
             * Mid or Unkown: Off. All power set to 0
             * Down: Pitch enabled, yaw and chassis Beyblade
             */
            int stick = remote.rightY();
            if (remote.rightSwitch() == Remote::SwitchState::UP){          // All non-serializer motors activated
                led3 = 1;
                unsigned long time = us_ticker_read();
                Chassis.setSpeedFF_Ks(0.065);
Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy *
                                          Chassis.m_OmniKinematicsLimits.max_Vel,
                                          0 * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                          ChassisSubsystem::YAW_ORIENTED);

                lastTime = time;

                // check switch mode
                // ground level = -5.69
                // lower bound = 15
                // upper bound = -25
                if (desiredPitch <= 15 and desiredPitch >= -25) {
                    desiredPitch += leftStickValue / 150;
                }
                else if (desiredPitch > 15 && leftStickValue < 0) {
                    desiredPitch = 15;
                }
                else if (desiredPitch < -25 && leftStickValue > 0) {
                    desiredPitch = -25;
                }
                float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
                pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
                pitch.setPosition(int((desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks)); 

                yawSetPoint -= remote.rightX() / 90;
                yawSetPoint = (yawSetPoint+360) % 360;
                timeSure = us_ticker_read();

                yaw.setSpeed(5 * yawNonBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure));
                imu.get_angular_position_quat(&imuAngles);

                prevTimeSure = timeSure;
                imu.get_angular_position_quat(&imuAngles);
            } else if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN){ // disable all the non-serializer components
                Chassis.setSpeedFF_Ks(0.065);
                Chassis.setWheelPower({0,0,0,0});
                yaw.setPower(0);
                pitch.setPower(0);

                yawSetPoint = (imuAngles.yaw + 180) ;
                        yawSetPoint = yawSetPoint % 360;
            } else if (remote.rightSwitch() == Remote::SwitchState::DOWN){           // beyblade mode
                unsigned long time = us_ticker_read(); //time for pid
                pitch.setPower(0);
                Chassis.setSpeedFF_Ks(0.065);
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          -RUNSPIN },ChassisSubsystem::YAW_ORIENTED);

                // check switch mode
                // ground level = -5.69
                // lower bound = 15
                // upper bound = -25
                if (desiredPitch <= 15 and desiredPitch >= -25) {
                    desiredPitch += leftStickValue / 150;
                }
                else if (desiredPitch > 15 && leftStickValue < 0) {
                    desiredPitch = 15;
                }
                else if (desiredPitch < -25 && leftStickValue > 0) {
                    desiredPitch = -25;
                }
                float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
                pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
                pitch.setPosition(int((desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks)); 

                yawSetPoint -= remote.rightX() / 90;
                yawSetPoint = (yawSetPoint+360) % 360;
                timeSure = us_ticker_read();
                yaw.setSpeed(-Chassis.getChassisSpeeds().vOmega * 8192 / 3.14 * 60 /8 + 15 * yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure));
                imu.get_angular_position_quat(&imuAngles);

                prevTimeSure = timeSure;
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
//             if (remote.rightSwitch() == Remote::SwitchState::UP)
//             { // All non-serializer motors activated

//                 double scalar = 1;
//                 double jx = remote.leftX() / 660.0 * scalar;
//                 double jy = remote.leftY() / 660.0 * scalar;
//                 double jr = remote.rightX() / 660.0 * scalar;

//                 double tolerance = 0.05;
//                 jx = (abs(jx) < tolerance) ? 0 : jx;
//                 jy = (abs(jy) < tolerance) ? 0 : jy;
//                 jr = (abs(jr) < tolerance) ? 0 : jr;

//                 chassis.setSpeedFF_Ks(0.065);
//                 chassis.setChassisSpeeds({jx * chassis.m_OmniKinematicsLimits.max_Vel,
//                                           jy * chassis.m_OmniKinematicsLimits.max_Vel,
//                                           -jr * chassis.m_OmniKinematicsLimits.max_vOmega},
//                                          ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);

//                 chassis.periodic();

//                 unsigned long time = us_ticker_read();
//             }
//             else if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN || remote.rightSwitch() == Remote::SwitchState::DOWN)
//             { // disable all the non-serializer components
//                 led3 = 0;
//                 chassis.setWheelPower({0, 0, 0, 0});
//                 yaw.setPower(0);
//                 pitch.setPower(0);
//                 pitch2.setPower(0);
//                 yawSetPoint = int(ext_game_robot_pos.data.yaw);
//                 pitchSetPosition();
//             }
//             yawTime = us_ticker_read();
//             if (remote.leftSwitch() == Remote::SwitchState::MID)
//             {
//                 gearSwap.setPower(-1500);
//                 double mps = ext_game_robot_state.data.shooter_id1_17mm_speed_limit * 0.9;
//                 double rpm = 60 * (mps / 0.03) / (3.14159 * 2);
//                 setFlyWheelSpeed(rpm);
//                 indexer.setPower(0);
//                 burstTimestamp = us_ticker_read();
//             }
//             else if (remote.leftSwitch() == Remote::SwitchState::DOWN || remote.leftSwitch() == Remote::SwitchState::UNKNOWN)
//             { // disable serializer
//                 indexer.setPower(0);
//                 setFlyWheelSpeed(0);
//             }
//             else if (remote.leftSwitch() == Remote::SwitchState::UP)
//             {
//                 gearSwap.setPower(-1500);
//                 double mps = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;
//                 double rpm = 60 * (mps / 0.03) / (3.14159 * 2);
//                 setFlyWheelSpeed(rpm); // was 0 * 0/
//                 if ((us_ticker_read() - burstTimestamp) / 1000 < 99)
//                 {
//                     indexer.setPower(8000);
//                 }
//                 else
//                 {
//                     indexer.setPower(0);
//                 }
                }
            } else {
                //SwitchState state set to mid/down/unknown
                shootReady = true;
            }
            if (remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN){
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

