#ifndef TR_EMBEDDED_HERO_H
#define TR_EMBEDDED_HERO_H
#include "main.h"
#include "subsystems/ChassisSubsystem.h"

// Add Hero constants here

//CONSTANTS
constexpr float LOWERBOUND = 12.0;
constexpr float UPPERBOUND = -25.0;

constexpr float BEYBLADE_OMEGA = 3.0;

constexpr int OUTER_LOOP_DT_MS = 15;

constexpr int PRINT_FREQUENCY = 20; //the higher the number, the less often

constexpr float CHASSIS_FF_KICK = 0.065;

#define USE_IMU

//CHASSIS DEFINING
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2794); // radius is 9 in

DJIMotor yaw(1, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(5, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right
DJIMotor indexer(2, CANHandler::CANBUS_2, C610,"Indexer");
DJIMotor RFLYWHEEL(4, CANHandler::CANBUS_2, M3508,"RightFly");
DJIMotor LFLYWHEEL(1, CANHandler::CANBUS_2, M3508,"LeftFly");
DJIMotor feeder(5, CANHandler::CANBUS_2, C610);

// robot specific sillies (gonna be deprecated)
int yawVelo = 0;
PID yawBeyblade(1,0.005,150);
float pitch_phase_angle = 33 / 180.0 * PI; // 5.69 theoretical //wtf is this?
float pitch_zero_offset_ticks = 2000;
float K = 0.38; // 0.75 //0.85
PID sure(0.5,0,0.4);
ChassisSpeeds cs;
Remote::SwitchState previous_mode = Remote::SwitchState::UNKNOWN;
bool prevM = false;

//Variables for burst fire
unsigned long shootTimer;

void init() {
    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();
    usbSerial.set_blocking(false);

    /*
    * MOTORS SETUP AND PIDS
    */
    //YAW
    yawBeyblade.setIntegralCap(2);
    yaw.setSpeedPID(1300, 1.1, 0);
    yaw.setSpeedIntegralCap(8000);
    yaw.setSpeedOutputCap(32000);
    yaw.outputCap = 16000;
    yaw.useAbsEncoder = false;

    #ifdef USE_IMU
    // imu.get_angular_position_quat(&imuAngles);
    while(imu.chip_ready()) {}
    imu.get_euler_angles((BNO055_EULER_TypeDef*)&imuAngles);
   
    imuAngles.yaw = 180 - imuAngles.yaw;
    imuAngles.pitch = 180 - imuAngles.pitch;
    imuAngles.roll = 180 - imuAngles.roll;
    yaw_desired_angle = imuAngles.yaw + 180;
    #else
    yaw_desired_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    #endif

    //PITCH
    pitch.setPositionPID(8, 0, 0); //15, 0, 1700
    pitch.setPositionOutputCap(32000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 16000;
    pitch.useAbsEncoder = true;

    //FLYWHEELS
    LFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);

    feeder.setSpeedPID(4, 0, 1);

    //INDEXER
    indexer.setSpeedPID(1.65, 0, 1);
    indexer.setSpeedIntegralCap(8000);
    //Cascading PID for indexer angle position control. Surely there are better names then "sure"...
    sure.setOutputCap(4000);

    //CHASSIS
    Chassis.setYawReference(&yaw, 2500); //the number of ticks of yaw considered to be robot-front
    //Common values for reference are 6500 and 2500
    Chassis.setSpeedFF_Ks(CHASSIS_FF_KICK); //feed forward "kick" for wheels, a constant multiplier of max power in the direcion of movment

}

void refthread() {
    while(1) {
        mutex_test.lock();

        timeStartRef = us_ticker_read();

        //referee loop every 15ms - seems like 6ms when dc and 600us when connected
        if ((timeStart - loopTimerRef) / 1000 > OUTER_LOOP_DT_MS){ 
            loopTimerRef = timeStart;
            led2 = referee.readable();
            referee.refereeThread();

            //POWER LIMIT OVERRIDE INCASE
            if(robot_status.chassis_power_limit < 10){
                chassis_power_limit = 49;
            }else{
                chassis_power_limit = robot_status.chassis_power_limit;
            }
            
            Chassis.power_limit = (float)chassis_power_limit;
        }

        mutex_test.unlock();
        ThisThread::sleep_for(1ms);
    }
}

void imuthread() {
    while(1) {
        mutex_test.lock();

        timeStartImu = us_ticker_read();

        if ((timeStartImu - loopTimerImu) / 1000 > 10){ 
            loopTimerImu = timeStartImu;
            
            #ifdef USE_IMU
            //imu.get_angular_position_quat(&imuAngles);
                imu.get_euler_angles((BNO055_EULER_TypeDef*)&imuAngles);
                imuAngles.yaw = 180 - imuAngles.yaw;
                imuAngles.pitch = 180 - imuAngles.pitch;
                imuAngles.roll = 180 - imuAngles.roll;
            #else
                yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
            #endif
        }

        mutex_test.unlock();
        ThisThread::sleep_for(1ms);
    }
}

void chassis_executor() {
    //Chassis Code
    if (drive == 'u' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::UP)){
        //REGULAR DRIVING CODE
        Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                    jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                    0 * Chassis.m_OmniKinematicsLimits.max_vOmega},
                                    ChassisSubsystem::YAW_ORIENTED);
    }else if (drive == 'd' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::DOWN)){
        //BEYBLADE DRIVING CODE
        Chassis.setChassisSpeeds({jx * Chassis.m_OmniKinematicsLimits.max_Vel,
                                    jy * Chassis.m_OmniKinematicsLimits.max_Vel,
                                    -BEYBLADE_OMEGA},
                                    ChassisSubsystem::YAW_ORIENTED);
    }else{
        //OFF
        Chassis.setWheelPower({0,0,0,0});
    }
}

void gimbal_executor() {
    //YAW CODE
    if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
        float chassis_rotation_radps = cs.vOmega;
        int chassis_rotation_rpm = chassis_rotation_radps * 60 / (2*M_PI) * 1.5; //I added this 4 but I don't know why.

        //Regular Yaw Code
        yaw_desired_angle -= jyaw * MOUSE_SENSITIVITY_YAW_DPS * elapsedms / 1000;
        yaw_desired_angle -= jyaw * JOYSTICK_SENSITIVITY_YAW_DPS * elapsedms / 1000;
        //yaw_desired_angle = (yaw_desired_angle + 360) % 360;
        yaw_desired_angle = floatmod(yaw_desired_angle, 360);

        prevTimeSure = timeSure;
        timeSure = us_ticker_read();
        #ifdef USE_IMU
        yawVelo = yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, imuAngles.yaw + 180, 360), timeSure - prevTimeSure);
        #else
        yawVelo = -jyaw * JOYSTICK_SENSITIVITY_YAW_DPS / 360.0 * 60;
        //yawVelo = yawBeyblade.calculatePeriodic(DJIMotor::s_calculateDeltaPhase(yaw_desired_angle, yaw_current_angle, 360), timeSure - prevTimeSure);
        #endif
        yawVelo -= chassis_rotation_rpm;
        //yawVelo *= 6; // scaled up arbitrarily 

        int dir = 0;
        if(yawVelo > 1){
            dir = 1;
        }else if(yawVelo < -1){
            dir = -1;
        }
        yaw.pidSpeed.feedForward = dir * (1855 + abs(yawVelo) * 120.48);
        yaw.setSpeed(yawVelo);
    }else{
        //Off
        yaw.setPower(0);
        #ifdef USE_IMU
        yaw_desired_angle = imuAngles.yaw + 180;
        #endif
    }

    //PITCH
    if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
        //Regular Pitch Code
        pitch_desired_angle += -jpitch * MOUSE_SENSITIVITY_PITCH_DPS * elapsedms / 1000;
        pitch_desired_angle -= -jpitch * JOYSTICK_SENSITIVITY_PITCH_DPS * elapsedms / 1000;

        if (pitch_desired_angle >= LOWERBOUND) {
            pitch_desired_angle = LOWERBOUND;
        }
        else if (pitch_desired_angle <= UPPERBOUND) {
            pitch_desired_angle = UPPERBOUND;
        }

        //float FF = K * sin((desiredPitch / 180 * PI) - pitch_phase); // output: [-1,1]
        //float FF = K * cos(pitch_desired_angle / 180 * PI);
        //pitch.pidPosition.feedForward = int((INT16_T_MAX) * FF);
        pitch.setPosition(int((pitch_desired_angle / 60) * TICKS_REVOLUTION + pitch_zero_offset_ticks));
    }else{
        //Off
        pitch.setPower(0);
    }
}

void shoot_executor() {
    //INDEXER CODE
    if((previous_mode == Remote::SwitchState::MID && remote.leftSwitch() == Remote::SwitchState::UP) || (!prevM && remote.getMouseL())){
        if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_42mm_barrel_heat < robot_status.shooter_barrel_heat_limit - 110) {
            shootTimer = us_ticker_read()/1000;
        }
    }

    if (us_ticker_read()/1000 - shootTimer < 200){
        feeder.setSpeed(7000);
    } else {
        feeder.setSpeed(0);
    }
    if (us_ticker_read()/1000 - shootTimer < 500){
        // indexer.setSpeed(8000);
        indexer.setSpeed(6000);
    }else if (us_ticker_read()/1000 - shootTimer > 500 && us_ticker_read()/1000 - shootTimer < 650){
        // indexer.setSpeed(8000);
        indexer.setPower(-16000);
    } else {
        indexer.setPower(0);
        // indexerOn = true;
    }
    
    //FLYWHEELS
    if (remote.leftSwitch() != Remote::SwitchState::DOWN &&
        remote.leftSwitch() != Remote::SwitchState::UNKNOWN &&
        remote.rightSwitch() != Remote::SwitchState::MID){
        RFLYWHEEL.setSpeed(-7475);
        LFLYWHEEL.setSpeed(7475);
    } else{
        // left SwitchState set to up/mid/unknown
        RFLYWHEEL.setSpeed(0);
        LFLYWHEEL.setSpeed(0);
    }
}

#endif //TR_EMBEDDED_HERO_H
