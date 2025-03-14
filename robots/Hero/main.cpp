// // increase speed --> p
// // avoid swing --> d

// // choose a p (< 50) and then tune d

// #include "main.h"
// #include "subsystems/ChassisSubsystem.h"
// #include <cmath>

// DigitalOut led(L27);
// DigitalOut led2(L26);
// DigitalOut led3(L25);

// #define JOYSTICK_SENSE_YAW 1.0/90
// #define JOYSTICK_SENSE_PITCH 1.0/150
// #define MOUSE_SENSE_YAW 1.0/3
// #define MOUSE_SENSE_PITCH 1.0/5
// #define MOUSE_KB_MULT 0.2

// I2C i2c(I2C_SDA, I2C_SCL);
// BNO055 imu(i2c, IMU_RESET, MODE_IMU);

// DJIMotor yaw(6, CANHandler::CANBUS_1, GM6020, "YAW");
// DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY, "PITCH");

// DJIMotor indexer(2, CANHandler::CANBUS_2, M3508, "INDEXER");
// DJIMotor feeder(5,CANHandler::CANBUS_2, C610, "FEEDER");

// DJIMotor top_flywheel(4,CANHandler::CANBUS_2, M3508_FLYWHEEL, "Top Flywheel");
// DJIMotor bottom_flywheel(1,CANHandler::CANBUS_2, M3508_FLYWHEEL, "Bottom Flywheel");

// ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.29845); // radius is 9 in



// int main(){

//     DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
//     DJIMotor::s_sendValues();
//     DJIMotor::s_getFeedback();

//     Chassis.setYawReference(&yaw, 2050); // "5604" is the number of ticks of yawOne considered to be robot-front
//     Chassis.setSpeedFF_Ks(0.065);

//     // yaw.useAbsEncoder = true;
//     yaw.setSpeedPID(10, 0, 0);
//     yaw.setSpeedOutputCap(32000);

//     // pitch.useAbsEncoder = true;
//     pitch.setPositionPID(10,0,5);
//     pitch.setPositionOutputCap(16000);
//     // pitch.useAbsEncoder = true;
//     int pitchUpperBound = 2000;
//     int pitchLowerBound = 7000;

//     long burstTimestamp = 0;

//     top_flywheel.setSpeedPID(1.5,0,0);
//     bottom_flywheel.setSpeedPID(1.5,0,0);

//     indexer.setSpeedPID(3,0,0);

//     unsigned long start = us_ticker_read();
//     unsigned long current = us_ticker_read();

//     unsigned long timeStart;
//     unsigned long loopTimer = us_ticker_read();

//     unsigned long power = 0;
//     int refLoop = 0;
//     int burstTimer = 0;

//     while (true)
//     {
//         timeStart = us_ticker_read();

//         if ((timeStart - loopTimer) / 1000 > 10)
//         {
//             led2 = !led2;

//             refLoop++;
//             if (refLoop >= 10){              // prints in here, prints less often
//                 refereeThread(&referee);
//                 refLoop = 0;
//                 led2 = !led2;

//                 // float pComponent = pitch.pidPosition.pComp;
//                 // float iComponent = pitch.pidPosition.iComp;
//                 // float dComponent = pitch.pidPosition.dComp;
//                 // printff("P = %f, I = %f, D = %f\n", pComponent, iComponent, dComponent);

//                 // printff("Yaw_POS: %d\n", yaw>>ANGLE);
//                 // printff("burst %d\n", us_ticker_read()-burstTimestamp);
//             }

//             remoteRead(); 


//             //MOVEMENT CODE BEGIN
//             double scalar = 1;
//             double jx = remote.leftX() / 660.0 * scalar;
//             double jy = remote.leftY() / 660.0 * scalar;
//             double jr = remote.rightX() / 660.0 * scalar;

//             double tolerance = 0.05;
//             jx = (abs(jx) < tolerance) ? 0 : jx;
//             jy = (abs(jy) < tolerance) ? 0 : jy;
//             jr = (abs(jr) < tolerance) ? 0 : jr;

//             // jx = MOUSE_KB_MULT * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
//             // jy = MOUSE_KB_MULT * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));
//             // jr = MOUSE_KB_MULT * ((remote.keyPressed(Remote::Key::E) ? 1 : 0) + (remote.keyPressed(Remote::Key::Q) ? -1 : 0));

//             if (remote.rightSwitch() == Remote::SwitchState::UP)
//             {           
//                 Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel, 
//                                           jy * Chassis.m_OmniKinematicsLimits.max_Vel, 
//                                           -jr * Chassis.m_OmniKinematicsLimits.max_vOmega}, 
//                                           ChassisSubsystem::ROBOT_ORIENTED);
//                 yaw.setPower(0);
//             }
//             else if (remote.rightSwitch() == Remote::SwitchState::DOWN)
//             {
//                 Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel, 
//                                           jy * Chassis.m_OmniKinematicsLimits.max_Vel, 
//                                           0}, 
//                                           ChassisSubsystem::ROBOT_ORIENTED);
//                 yaw.setSpeed(int(-jr * 300));  
//             }
//             else
//             {
//                 Chassis.setChassisSpeeds({0, 0, 0});
//                 yaw.setPower(0);
//             }

//             if(remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN){
//                 int mid = (pitchLowerBound + pitchUpperBound)/2;
//                 int range = pitchLowerBound - pitchUpperBound;
//                 pitch.setPosition(mid + (remote.rightY()/-660.0 * range));
//             }else{
//                 pitch.setPower(0);
//             }
            
//             Chassis.periodic();
//             //MOVEMENT CODE END
            
//             #define feederburstlength 100
//             #define burstthreshold 0
//             #define indexerburstend 1000
//             //SHOOTING CODE BEGIN
//             double mps = ext_game_robot_state.data.shooter_id1_42mm_speed_limit;
//             double rpm = 60 * (mps / 0.03) / (3.14159 * 2);

//             if(mps == 0) rpm = 5400;
//             // printff("rpm: %f\n",rpm);
//             rpm = 4000;
            
//             // shooting code
//             if (remote.leftSwitch() == Remote::SwitchState::UP ) {
//                 // printff("shootin' ");
//                 //pitch.setPower(leftStickValue / 20);

//                 top_flywheel.setSpeed(rpm);
//                 bottom_flywheel.setSpeed(-rpm);


//                 burstTimer = (us_ticker_read() - burstTimestamp)/1000;
//                 if(burstTimer < feederburstlength){
//                     feeder.setPower(8000);
//                 }else{
//                     feeder.setPower(0);
//                 }
//                 if(burstTimer < indexerburstend){
//                     indexer.setSpeed(70 * M3508_GEAR_RATIO);
//                 }else{
//                     indexer.setPower(0);
//                 }
//             }
//             else if (remote.leftSwitch() == Remote::SwitchState::MID ) {
//                 // printff("revvin' up @%fRPM", rpm);
//                 // pitch.setPower(0);
//                 top_flywheel.setSpeed(rpm*0.9);
//                 bottom_flywheel.setSpeed(-rpm*0.9);
//                 burstTimestamp = us_ticker_read();
//                 indexer.setPower(0);
//                 feeder.setPower(0);
//             }else{
//                 // printff("naaaaah ");
//                 top_flywheel.setSpeed(0);
//                 bottom_flywheel.setSpeed(0);
//                 if(abs(top_flywheel>>VELOCITY) < 500) top_flywheel.setPower(0);
//                 if(abs(bottom_flywheel>>VELOCITY) < 500) bottom_flywheel.setPower(0);
//                 burstTimestamp = us_ticker_read();
//             }

//             // printff("burst %d\n", (us_ticker_read()-burstTimestamp)/1000);
//             // int burstTimer = (us_ticker_read() - burstTimestamp)/1000;
//             // if(burstTimer > burstthreshold){
                
//             // }
//             // if(burstTimer >= feederburstlength){
//             //     feeder.setPower(0);
//             // }
//             // if(burstTimer >= feederburstlength){
//             //     indexer.setPower(0);
//             // }
//             //SHOOTING CODE END
            
//             loopTimer = timeStart;
//             DJIMotor::s_sendValues();

//         }
//         DJIMotor::s_getFeedback();
//         ThisThread::sleep_for(1ms);
//     }
// }


#include "main.h"
#include "Hero.h"
// #include <jetson.h>
#include <cstdlib>

#include <iostream>
#include "subsystems/ChassisSubsystem.h"

#define PI 3.14159265

#define LOWERBOUND 11.0 * 6
#define UPPERBOUND -35.0 * 6

// add radius measurement here
#define RADIUS 0.5
#define RUNSPIN 1.0

#define JOYSTICK_SENSE_YAW 1.0/180
#define JOYSTICK_SENSE_PITCH 1.0/60
#define MOUSE_SENSE_YAW 1.0/4.5
#define MOUSE_SENSE_PITCH 1.0/1.5
#define MOUSE_KB_MULT 0.2

I2C i2c(I2C_SDA, I2C_SCL);

BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY); // right
// DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left, not functioning

DJIMotor feeder(5, CANHandler::CANBUS_2, C610);
DJIMotor indexer(2, CANHandler::CANBUS_2, M3508);
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_2, M3508);

DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);
DigitalOut led4(LED1);
DigitalIn button(BUTTON1);
DigitalIn jumper(PC_9);

BNO055_ANGULAR_POSITION_typedef imuAngles;

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

int main()
{

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);

    /*
    * MOTORS SETUP AND PIDS
    */

    //TODO: tune pitch pid
//    pitch.setPositionPID(1, 0.01, 200); //12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
//    pitch.setPositionIntegralCap(6000);
  //   merge difference:
//     pitch.setPositionPID(17.3, 0.03, 8.5); // 12.3 0.03 2.5 //mid is 13.3, 0.03, 7.5
//     pitch.setPositionIntegralCap(60000);
//    pitch.setPositionOutputCap(100000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 32760;
    pitch.useAbsEncoder = true;

    //Variables & PID for burst fire
    bool shoot = false;
    //int shootPosition;
    int shootTargetPosition = 36*8190 ;
    unsigned long shootTimer;
    bool shootReady = false;

    char drive = 'o';

    // pitch.useAbsEncoder = true;
    pitch.setPositionPID(10, 0, 70); //15, 0 1700
    pitch.setPositionOutputCap(32000);
    float currentPitch = 0;
    float desiredPitch = 0;
    float pitch_phase = 33 / 180.0 * PI; // 5.69 theoretical
    float InitialOffset_Ticks = 1400;
    float K = 0.38; // 0.75 //0.85

    LFLYWHEEL.setSpeedPID(7.5, 0, 0.04);
    RFLYWHEEL.setSpeedPID(7.5, 0, 0.04);

    feeder.setSpeedPID(4, 0, 1);
    //     merge difference
    //     PID yawIMU(200.0, 0.1, 150, 20000, 8000); // 7.0,0.02,15.0,20000,8000
    Chassis.setYawReference(&yaw, 2050+ 2048*3+ 4096);
    Chassis.setSpeedFF_Ks(0.065);

    yaw.setSpeedPID(0.3, 0, 150);
    PID yawBeyblade(40, 0, 5);
    PID yawNonBeyblade(80, 0, 50);

    yaw.setSpeedIntegralCap(1000);
    yaw.useAbsEncoder = false;

    indexer.setSpeedPID(2, 0, 0);
    indexer.setSpeedIntegralCap(8000);
    //PID for indexer angle position control. Surely there are better names then "sure"...
    PID sure(0.5,0,0.4);
    sure.setOutputCap(4000);
    unsigned long timeSure;
    unsigned long prevTimeSure;
    //  merge difference
    //   chassis.setBrakeMode(ChassisSubsystem::BrakeMode::COAST);

    // command.initialize();

    /*
    * OPERATIONAL VARIABLES
    */
    unsigned long loopTimer = us_ticker_read();

    int refLoop = 0;
    int ref_yaw;

    int yawSetPoint = imuAngles.yaw;
    double rotationalPower = 0;

    jumper.mode(PullDown);

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



    bool userButton;
    bool prev_userButton;
    char driveMode = 'j';

    bool prev_R = false;
    bool key_R = false;

    float angleOffset = 0.0;

    imu.get_angular_position_quat(&imuAngles);

    yawSetPoint = (imuAngles.yaw + 180) ;
    yawSetPoint = yawSetPoint % 360;

    float p_theory_tot_c = 0;

    while (true)
    {
        unsigned long timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 15){
            led3 = !led3;

            loopTimer = timeStart;
            prev_R = key_R;
            remoteRead();
            key_R = remote.keyPressed(Remote::Key::R);
            
            // key_R = remote.keyPressed(Remote::Key::R);
            Chassis.periodic();

            if(jumper){
                driveMode = 'm';
                // led = 1;
            }else{
                driveMode = 'j';
                // led = 0;
            }
            if(remote.keyPressed(Remote::Key::B)){
                drive = 'm';
            }else if(remote.keyPressed(Remote::Key::E)){
                drive = 'u';
            }else if(remote.keyPressed(Remote::Key::Q)){
                drive = 'd';        
            }

            refLoop++;
            imu.get_angular_position_quat(&imuAngles);

            float angle = -(yaw>>MULTITURNANGLE);
            while(angle < 0){
                angle += TICKS_REVOLUTION * 2;
            }
            while(angle > TICKS_REVOLUTION * 2){
                angle -= TICKS_REVOLUTION * 2;
            }
            angle /= 2;
            float currentAngle = ((1.0 - (angle / TICKS_REVOLUTION)) * 360.0 - angleOffset + 180);
            if(!prev_R && key_R){
                led = 1;
                angleOffset += 180;
                if(angleOffset > 360){
                    angleOffset -= 360;
                }
            }
            while(currentAngle > 360){
                currentAngle -= 360;
            }
            while(currentAngle < 0){
                currentAngle += 360;
            }

            

            // imu.get_angular_position_quat(&imuAngles);

            if(remote.keyPressed(Remote::Key::F)){
                angleOffset = (1.0 - (angle / TICKS_REVOLUTION)) * 360.0;
            }

            if (refLoop >= 10){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
                // led4 = button;

                // printff("%f %d %d %d\n", imuAngles.yaw, yawSetPoint, remote.getMouseX()*MOUSE_SENSE_YAW, yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure));
                // printff("ang%f t%d d%f FF%f\n", (((pitch>>ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360, pitch>>ANGLE, desiredPitch, K * sin((desiredPitch / 180 * PI) - pitch_phase)); //(desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks
//                printff("%d %d %d %d [%f]\n"
//                    ,Chassis.getMotor(ChassisSubsystem::LEFT_FRONT)>>POWEROUT
//                    ,Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT)>>POWEROUT
//                    ,Chassis.getMotor(ChassisSubsystem::LEFT_BACK)>>POWEROUT
//                    ,Chassis.getMotor(ChassisSubsystem::RIGHT_BACK)>>POWEROUT
//                    ,currentAngle);
                // printff("i: %f, p: %d, p_P: %d \n", imuAngles.yaw, pitch>>ANGLE, pitch>>POWEROUT);
                mbed_reset_reboot_count();
                
                if(robot_status.chassis_power_limit < 10){
                    Chassis.PEAK_POWER_ALL = 32000;
                }else{
                    Chassis.PEAK_POWER_ALL = 200 * robot_status.chassis_power_limit;
                }
                
                //printff("%.3f\t%.3f\t%d\n", p_theory_tot_c, power_heat_data.chassis_power, 60);
                printff("%d %d %d %d\n", RFLYWHEEL>>ANGLE, LFLYWHEEL>>ANGLE, pitch>>ANGLE, feeder>>ANGLE);
                                     
                // printff("%d %d %.1f %d\n", remote.rightX(), DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), imuAngles.yaw + 180, yaw>>POWEROUT);
//                printff("\n");

                // printff("i: %f, p: %d, p_P: %d \n", imuAngles.yaw, pitch>>ANGLE, pitch>>POWEROUT);
                //printff("ang%f t%d d%f FF%f\n", (((pitch>>ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360, pitch>>ANGLE, desiredPitch, K * sin((desiredPitch / 180 * PI) - pitch_phase)); //(desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks
                // printff("%d %d %d %d\n", robot_status.robot_id, robot_status.robot_level, robot_status.current_HP, robot_status.maximum_HP);
                // printff("%d %d %d %d %c\n", robot_status.chassis_power_limit, robot_status.shooter_barrel_heat_limit, currentAngle, robot_status.maximum_HP, drive);
                
                
                // printff("%.1f %d %d %d\n", currentAngle, yawSetPoint, prev_R, key_R);
            
            }

            if(remote.keyPressed(Remote::Key::CTRL)){
                    Chassis.PEAK_POWER_ALL = 32000;
            }
            else if(remote.keyPressed(Remote::Key::G)){
                    Chassis.PEAK_POWER_ALL = 60000;
            }

            double scalar = 1;
            double jx = remote.leftX() / 660.0 * scalar;
            double jy = remote.leftY() / 660.0 * scalar;
            double jr = remote.rightX() / 660.0 * scalar;

            double tolerance = 0.05;
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jr = (abs(jr) < tolerance) ? 0 : jr;

            float mult = 1;
            if(remote.keyPressed(Remote::Key::SHIFT)){
                mult = 0.5;
            }

            jx += mult * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
            jy += mult * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));
            jr += mult * ((remote.keyPressed(Remote::Key::E) ? 1 : 0) + (remote.keyPressed(Remote::Key::Q) ? -1 : 0));

            currentPitch = (double(pitch.getData(ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360; // degrees

            int leftStickValue = remote.rightY();

            /**
             * RightSwitch controls: Pitch, Yaw, Chassis
             * Up: Pitch enabled, yaw and chassis seperate
             * Mid or Unkown: Off. All power set to 0
             * Down: Pitch enabled, yaw and chassis Beyblade
             */
            int stick = remote.rightY();
            if (drive == 'u' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::UP)){          // All non-serializer motors activated
                // led3 = 1;
                unsigned long time = us_ticker_read();
                Chassis.setSpeedFF_Ks(0.065);
                ChassisSpeeds cs = Chassis.rotateChassisSpeed({jx * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          0 * Chassis.m_OmniKinematicsLimits.max_vOmega}, 
                                          currentAngle // change Yaw to CCW +, and ranges from 0 to 360
                );
                Chassis.setChassisSpeeds(cs,
                                          ChassisSubsystem::ROBOT_ORIENTED);


                    
     

                lastTime = time; 

                yawSetPoint -= remote.getMouseX() * MOUSE_SENSE_YAW;
                if(remote.rightX() > 55 || remote.rightX() < -5){
                    yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;
                }
                yawSetPoint = (yawSetPoint+360) % 360;

                timeSure = us_ticker_read();

                yaw.setSpeed(2 * 5 * yawNonBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure));


                prevTimeSure = timeSure;
                // imu.get_angular_position_quat(&imuAngles);
            } else if (drive == 'm' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN)){ // disable all the non-serializer components
                Chassis.setSpeedFF_Ks(0.065);
                Chassis.setWheelPower({0,0,0,0});
                yaw.setPower(0);

                yawSetPoint = (imuAngles.yaw + 180) ;
                yawSetPoint = yawSetPoint % 360;
            } else if (drive == 'd' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::DOWN)){           // beyblade mode
                unsigned long time = us_ticker_read(); //time for pid
//                pitch.setPower(0);
                Chassis.setSpeedFF_Ks(0.065);
                // Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                //                           jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                //                           -RUNSPIN },ChassisSubsystem::YAW_ORIENTED);

                ChassisSpeeds cs = Chassis.rotateChassisSpeed({jx * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel, 
                                          -RUNSPIN}, 
                                          currentAngle // change Yaw to CCW +, and ranges from 0 to 360
                );
                Chassis.setChassisSpeeds(cs, 
                                          ChassisSubsystem::ROBOT_ORIENTED);

                // if(driveMode == 'm'){
                //     yawSetPoint -= remote.getMouseX() * MOUSE_SENSE_YAW;
                // }else{
                //     yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;
                // }

                yawSetPoint -= remote.getMouseX() * MOUSE_SENSE_YAW;
                // yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;
                if(remote.rightX() > 45 || remote.rightX() < -5){
                    yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;
                }

                yawSetPoint = (yawSetPoint+360) % 360;
                
                timeSure = us_ticker_read();

                int yawVelo = 2 * 5 * yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure);

                yaw.setSpeed(-2 * Chassis.getChassisSpeeds().vOmega * 8192 / 3.14 * 30 + yawVelo);
                imu.get_angular_position_quat(&imuAngles);

                prevTimeSure = timeSure;
            }
            yawTime = us_ticker_read();

            int t1 = abs(Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(POWEROUT));
            int t2 = abs(Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(POWEROUT));
            int t3 = abs(Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(POWEROUT));
            int t4 = abs(Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(POWEROUT));

            int r1 = abs(Chassis.getMotor(ChassisSubsystem::LEFT_FRONT).getData(VELOCITY));
            int r2 = abs(Chassis.getMotor(ChassisSubsystem::RIGHT_FRONT).getData(VELOCITY));
            int r3 = abs(Chassis.getMotor(ChassisSubsystem::LEFT_BACK).getData(VELOCITY));
            int r4 = abs(Chassis.getMotor(ChassisSubsystem::RIGHT_BACK).getData(VELOCITY));

            p_theory_tot_c = ChassisSubsystem::p_theory(t1, t2, t3, t4, r1, r2, r3, r4);

            // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
            // only shoot when left switch changes from down/unknown/mid to up
            // if left switch remains at up state, indexer stops after 3-5 balls
             if (shoot){
//                 if (indexer>>MULTITURNANGLE >= shootTargetPosition){
//                     // indexer.setSpeed(0);
//                     shoot = false;
//                 } else {
//                     timeSure = us_ticker_read();
//                     // indexer.setSpeed(0); //
//                     // prevTimeSure = timeSure;
//                 }
                //feeder
                bool feederOn = false;
                bool indexerOn = false;
                if (us_ticker_read()/1000 - shootTimer < 140){
                    feeder.setSpeed(7000);
                } else {
                    feeder.setSpeed(0);
                    feederOn = true;
                }
                //indexer
                if (us_ticker_read()/1000 - shootTimer < 300){
                    indexer.setSpeed(5000);
                } else {
                    indexer.setSpeed(100);
                    indexerOn = true;
                }
                if (indexerOn && feederOn){
                    shoot = false;
                }

             } else {
                indexer.setSpeed(200);
//                 feeder.setSpeed(0);
                feeder.setPower(0);
             }

            //PITCH CODE, if remote is UP or DOWN, run pitch code, else off
            if (drive == 'u' || drive == 'd' || (drive == 'o' && (remote.rightSwitch() == Remote::SwitchState::UP ||
                remote.rightSwitch() == Remote::SwitchState::DOWN))){

                // check switch mode
                // ground level = -5.69
                // lower bound = 15
                // upper bound = -25
                
                // printff("i%f\n",desiredPitch);
                // if(driveMode == 'm'){
                //     desiredPitch += remote.getMouseY() * MOUSE_SENSE_PITCH;
                // }else{
                //     desiredPitch -= leftStickValue * JOYSTICK_SENSE_PITCH;
                // }

                desiredPitch += remote.getMouseY() * MOUSE_SENSE_PITCH;
                desiredPitch += leftStickValue * JOYSTICK_SENSE_PITCH;

                if (desiredPitch >= LOWERBOUND) {
                    // printff("u%f\n",desiredPitch);
                    desiredPitch = LOWERBOUND;
                }
                else if (desiredPitch <= UPPERBOUND) {
                    // printff("d%f\n",desiredPitch);
                    desiredPitch = UPPERBOUND;
                }

                float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
//                pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
//                    pitch.setPower(0);
                pitch.setPosition(int((desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks));

            } else{
                pitch.setPower(0);
            }




            /**
             * left switch controls burst fire: indexer and flywheels
             * state down or unknown: no motion
             * state mid: flywheels running, resetting shoot state
             * state up: flywheels continues running, turn indexer if state set by state mid
             */
            if (remote.leftSwitch() == Remote::SwitchState::UP  || (remote.getMouseL() && remote.leftSwitch() == Remote::SwitchState::MID)){
                // Monitors state of left switch at previous loop and determine whether to turn indexer on
                // if left switch was at other states, turn indexer on
                // otherwise, continue the burstfire and stop after 3-5 shots
                // $shootReady local to if block, $shoot variable used above
//                feeder.setSpeed(1000);
                 if (shootReady){
                     shootReady = false;
                    if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_42mm_barrel_heat < robot_status.shooter_barrel_heat_limit - 110) {
                        shoot = true;
                    }else {
                        shoot = false;
                    }

//                     shoot = true;
                     shootTargetPosition = 8192 * 3 + (indexer>>MULTITURNANGLE);
//                    if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat < ext_game_robot_state.data.shooter_id1_17mm_cooling_limit - 40) {
//                    shoot = true;
                    shootTimer = us_ticker_read()/1000;
//                    }
                 }

//                indexer.setPower(5000);
//                feeder.setPower(1000);
            } else {
                //SwitchState state set to mid/down/unknown
                 shootReady = true;
                indexer.setPower(0);
//                feeder.setPower(0);
                feeder.setSpeed(0);
            }

            if (remote.leftSwitch() != Remote::SwitchState::DOWN &&
                remote.leftSwitch() != Remote::SwitchState::UNKNOWN){
                RFLYWHEEL.setSpeed(-5750);
                LFLYWHEEL.setSpeed(5750);
            } else{
                // left SwitchState set to up/mid/unknown
                RFLYWHEEL.setSpeed(0);
                LFLYWHEEL.setSpeed(0);
            }
            DJIMotor::s_sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::s_getFeedback();
        prev_userButton = userButton;
        
        ThisThread::sleep_for(1ms);
        userButton = button;
    }
}