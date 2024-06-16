#include "main.h"
#include "Infantry.h"
// #include <jetson.h>
#include <cstdlib>

#include <iostream>
#include "subsystems/ChassisSubsystem.h"

#define PI 3.14159265

#define LOWERBOUND 35.0
#define UPPERBOUND -15.0

// add radius measurement here
#define RADIUS 0.5
#define RUNSPIN 1.0

#define JOYSTICK_SENSE_YAW 1.0/90
#define JOYSTICK_SENSE_PITCH 1.0/150
#define MOUSE_SENSE_YAW 1.0/3
#define MOUSE_SENSE_PITCH 1.0/5
#define MOUSE_KB_MULT 0.2

I2C i2c(I2C_SDA, I2C_SCL);

BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 in
DJIMotor yaw(5, CANHandler::CANBUS_1, GIMBLY);
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY); // right
DJIMotor pitch2(6, CANHandler::CANBUS_2, GIMBLY); // left, not functioning

DJIMotor indexer(7, CANHandler::CANBUS_2, C610);
DJIMotor RFLYWHEEL(8, CANHandler::CANBUS_2, M3508);
DJIMotor LFLYWHEEL(5, CANHandler::CANBUS_2, M3508);

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
    pitch.setPositionPID(15, 0, 1700); //15, 0 1700
    pitch.setPositionOutputCap(32000);
    float currentPitch = 0;
    float desiredPitch = 0;
    float pitch_phase = 33 / 180.0 * PI; // 5.69 theoretical
    float InitialOffset_Ticks = 2500;
    float K = 0.38; // 0.75 //0.85

    LFLYWHEEL.setSpeedPID(7.5, 0, 0.04);
    RFLYWHEEL.setSpeedPID(7.5, 0, 0.04);

    //     merge difference
    //     PID yawIMU(200.0, 0.1, 150, 20000, 8000); // 7.0,0.02,15.0,20000,8000
    Chassis.setYawReference(&yaw, 4098); // "5604" is the number of ticks of yawOne considered to be robot-front
    Chassis.setSpeedFF_Ks(0.065);
    
    yaw.setSpeedPID(1.5, 0, 200);
    PID yawBeyblade(50, 0, 5);
    PID yawNonBeyblade(100, 0, 50);

    yaw.setSpeedIntegralCap(1000);
    yaw.useAbsEncoder = false;

    indexer.setSpeedPID(1, 0, 1);
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

    //Variables & PID for burst fire
    bool shoot = false;
    //int shootPosition;
    int shootTargetPosition = 36*8190 ;
    bool shootReady = false;

    char drive = 'o';
    char shot = 'o';

    bool userButton;
    bool prev_userButton;
    char driveMode = 'j';

    while (true)
    {
        unsigned long timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            led3 = !led3;

            loopTimer = timeStart;
            remoteRead();
            Chassis.periodic();

            if(jumper){
                driveMode = 'm';
                // led = 1;
            }else{
                driveMode = 'j';
                // led = 0;
            }
            if(remote.keyPressed(Remote::Key::R)){
                drive = 'm';
            }else if(remote.keyPressed(Remote::Key::E)){
                drive = 'u';
            }else if(remote.keyPressed(Remote::Key::Q)){
                drive = 'd';        
            }
            if(remote.keyPressed(Remote::Key::C)){
                shot = 'm';
            }else if(remote.keyPressed(Remote::Key::V)){
                shot = 'u';
            }else if(remote.keyPressed(Remote::Key::B)){
                shot = 'd';        
            }

            refLoop++;
            imu.get_angular_position_quat(&imuAngles);
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
                // led4 = button;
                // printff("%f %f %d \n", ext_power_heat_data.data.chassis_power, ext_power_heat_data.data.chassis_power_buffer, ext_game_robot_state.data.chassis_power_limit);
                // printff("%f %d %d %d\n", imuAngles.yaw, yawSetPoint, remote.getMouseX()*MOUSE_SENSE_YAW, yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure));
                // printff("ang%f t%d d%f FF%f\n", (((pitch>>ANGLE) - InitialOffset_Ticks) / TICKS_REVOLUTION) * 360, pitch>>ANGLE, desiredPitch, K * sin((desiredPitch / 180 * PI) - pitch_phase)); //(desiredPitch / 360) * TICKS_REVOLUTION + InitialOffset_Ticks
                
                if(remote.keyPressed(Remote::Key::E)){
                    Chassis.PEAK_POWER_ALL = 60000;
                    // led = 1;
                }else{
                    // led = 0;
                    if(robot_status.chassis_power_limit < 10){
                        Chassis.PEAK_POWER_ALL = 8000;
                    }else{
                        Chassis.PEAK_POWER_ALL = 141 * robot_status.chassis_power_limit;
                    }
                }

                // printff("%d %d \n" , ext_power_heat_data.data.shooter_id1_17mm_cooling_heat, ext_game_robot_state.data.shooter_id1_17mm_cooling_limit);
            }


            double scalar = 1;
            double jx = remote.leftX() / 660.0 * scalar;
            double jy = remote.leftY() / 660.0 * scalar;
            double jr = remote.rightX() / 660.0 * scalar;

            double tolerance = 0.05;
            jx = (abs(jx) < tolerance) ? 0 : jx;
            jy = (abs(jy) < tolerance) ? 0 : jy;
            jr = (abs(jr) < tolerance) ? 0 : jr;

            // if(driveMode == 'm'){
            //     jx = MOUSE_KB_MULT * ((remote.keyPressed(Remote::Key::D) ? 1 : 0) + (remote.keyPressed(Remote::Key::A) ? -1 : 0));
            //     jy = MOUSE_KB_MULT * ((remote.keyPressed(Remote::Key::W) ? 1 : 0) + (remote.keyPressed(Remote::Key::S) ? -1 : 0));
            //     jr = MOUSE_KB_MULT * ((remote.keyPressed(Remote::Key::E) ? 1 : 0) + (remote.keyPressed(Remote::Key::Q) ? -1 : 0));
            // }

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
                led3 = 1;
                unsigned long time = us_ticker_read();
                Chassis.setSpeedFF_Ks(0.065);
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy *
                                          Chassis.m_OmniKinematicsLimits.max_Vel,
                                          0 * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                          ChassisSubsystem::YAW_ORIENTED);

                lastTime = time; 

                // if(driveMode == 'm'){
                //     yawSetPoint -= remote.getMouseX() * MOUSE_SENSE_YAW;
                // }else{
                //     yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;
                // }

                yawSetPoint -= remote.getMouseX() * MOUSE_SENSE_YAW;
                yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;

                yawSetPoint = (yawSetPoint+360) % 360;

                timeSure = us_ticker_read();

                yaw.setSpeed(5 * yawNonBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yawSetPoint,imuAngles.yaw+180, 360), timeSure - prevTimeSure));
                imu.get_angular_position_quat(&imuAngles);

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
                pitch.setPower(0);
                Chassis.setSpeedFF_Ks(0.065);
                Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                          -RUNSPIN },ChassisSubsystem::YAW_ORIENTED);

                // if(driveMode == 'm'){
                //     yawSetPoint -= remote.getMouseX() * MOUSE_SENSE_YAW;
                // }else{
                //     yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;
                // }

                yawSetPoint -= remote.getMouseX() * MOUSE_SENSE_YAW;
                yawSetPoint -= remote.rightX() * JOYSTICK_SENSE_YAW;

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
                desiredPitch -= leftStickValue * JOYSTICK_SENSE_PITCH;

                if (desiredPitch >= LOWERBOUND) {
                    // printff("u%f\n",desiredPitch);
                    desiredPitch = LOWERBOUND;
                }
                else if (desiredPitch <= UPPERBOUND) {
                    // printff("d%f\n",desiredPitch);
                    desiredPitch = UPPERBOUND;
                }

                float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
                pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
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
            if (remote.leftSwitch() == Remote::SwitchState::UP || remote.getMouseL()){
                // Monitors state of left switch at previous loop and determine whether to turn indexer on
                // if left switch was at other states, turn indexer on
                // otherwise, continue the burstfire and stop after 3-5 shots
                // $shootReady local to if block, $shoot variable used above
                if (shootReady){
                    shootReady = false;
                    // shoot = true;
                    shootTargetPosition = 8192 * 12 + (indexer>>MULTITURNANGLE);

                    //shoot limit
                    if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_1_barrel_heat < robot_status.shooter_barrel_heat_limit - 40) {
                        shoot = true;
                    }
                    
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
        prev_userButton = userButton;
        ThisThread::sleep_for(1ms);
        userButton = button;
    }
}

