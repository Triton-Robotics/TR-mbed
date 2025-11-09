#ifndef TR_EMBEDDED_INFANTRY_H
#define TR_EMBEDDED_INFANTRY_H
#include "main.h"
#include "subsystems/ChassisSubsystem.h"
// Add Infantry constants here

//CONSTANTS
constexpr float LOWERBOUND = -35.0;
constexpr float UPPERBOUND = 40.0;

constexpr int OUTER_LOOP_DT_MS = 1;

constexpr float CHASSIS_FF_KICK = 0.065;

constexpr float pitch_zero_offset_ticks = 1500;

constexpr int NUM_BALLS_SHOT = 3;
constexpr int FLYWHEEL_VELO = 5500;

// Chassis Definitions
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.22617); // radius is 9 in
DJIMotor yaw(4, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right
DJIMotor indexer(7, CANHandler::CANBUS_2, C610,"Indexer");
DJIMotor RFLYWHEEL(1, CANHandler::CANBUS_2, M3508,"RightFly");
DJIMotor LFLYWHEEL(2, CANHandler::CANBUS_2, M3508,"LeftFly");

// robot specific sillies (gonna be deprecated)
int yawVelo = 0;
PID pitchCascade(1.5,0.0005,0.05);
PID sure(0.1,0,0.001);
ChassisSpeeds cs;


void shoot_executor() {

    //INDEXER CODE
    if ((remote.leftSwitch() == Remote::SwitchState::UP || remote.getMouseL()) && (abs(RFLYWHEEL>>VELOCITY) > (FLYWHEEL_VELO - 500) && abs(LFLYWHEEL>>VELOCITY) > (FLYWHEEL_VELO - 500)) 
        /*&& remote.rightSwitch() != Remote::SwitchState::MID*/){        
        if (shootReady){
    
            //shoot limit
            if(robot_status.shooter_barrel_heat_limit < 10 || power_heat_data.shooter_17mm_1_barrel_heat < robot_status.shooter_barrel_heat_limit - 40) {
                shoot = true;
                shootReady = false;
                shootTargetPosition = (8192 * M2006_GEAR_RATIO / 9 * NUM_BALLS_SHOT) + (indexer>>MULTITURNANGLE);
            }
            
        }
    } else if(!(remote.leftSwitch() == Remote::SwitchState::UP || remote.getMouseL())) {
        //SwitchState state set to mid/down/unknown
        shootReady = true;
    }
    
    // burst fire, turn the indexer to shoot 3-5 balls a time and stop indexer
    // only shoot when left switch changes from down/unknown/mid to up
    // if left switch remains at up state, indexer stops after 3-5 balls
    int indexer_target_velocity = 0; 
    if (shoot){
        // 1 degree of error allowed
        if (abs((indexer>>MULTITURNANGLE) - shootTargetPosition) <= 819){
            // indexer.setSpeed(indexer_target_velocity);
            // indexer.pidSpeed.feedForward = 0;
            shoot = false;
        } else {
            indexer_target_velocity = sure.calculate(shootTargetPosition, indexer>>MULTITURNANGLE, timeSure - prevTimeSure);
            indexer.setSpeed(indexer_target_velocity); //
            indexer.pidSpeed.feedForward = (indexer>>VALUE) / 4788 * 630;
        }
    } else {
        indexer.setSpeed(0);
        indexer.pidSpeed.feedForward = 0;
    }
    
    //FLYWHEELS
    if (shot == 'm' || (shot == 'o' && remote.leftSwitch() != Remote::SwitchState::DOWN && remote.leftSwitch() != Remote::SwitchState::UNKNOWN)){
        LFLYWHEEL.setSpeed(-FLYWHEEL_VELO);
        RFLYWHEEL.setSpeed(FLYWHEEL_VELO);
        LFLYWHEEL.pidSpeed.feedForward = 52;
        RFLYWHEEL.pidSpeed.feedForward = 77;
    } else{
        // left SwitchState set to up/mid/unknown
        if(abs(LFLYWHEEL>>VELOCITY) < 50){
            LFLYWHEEL.setPower(0);
        }else{
            LFLYWHEEL.setSpeed(0);
        }
        if(abs(RFLYWHEEL>>VELOCITY) < 50){
            RFLYWHEEL.setPower(0);
        }else{
            RFLYWHEEL.setSpeed(0);
        }
        LFLYWHEEL.pidSpeed.feedForward = 0;
        RFLYWHEEL.pidSpeed.feedForward = 0;
    }
}

void chassis_executor() {  
    ChassisSpeeds beybladeSpeeds = {jx * max_linear_vel,
                                  jy * max_linear_vel,
                                  -omega_speed};
    if (drive == 'u' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::UP)){
        //REGULAR DRIVING CODE
        Chassis.setChassisSpeeds({jx * max_linear_vel,
                                  jy * max_linear_vel,
                                  0},
                                  ChassisSubsystem::YAW_ORIENTED);
    }else if (drive == 'd' || (drive =='o' && remote.rightSwitch() == Remote::SwitchState::DOWN)){
        //BEYBLADE DRIVING CODE
        Chassis.setChassisSpeeds(beybladeSpeeds,
                                  ChassisSubsystem::YAW_ORIENTED);
    }else{
        //OFF
        Chassis.setWheelPower({0,0,0,0});
    }
}

void gimbal_executor() {
    //YAW CODE 
    float error = 0;
    if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
        float chassis_rotation_radps = cs.vOmega;
        int chassis_rotation_rpm = chassis_rotation_radps * 60 / (2*PI) * 1.5; //I added this 4 but I don't know why.
        
        #ifdef USE_IMU
        error = DJIMotor::s_calculateDeltaPhaseF(yaw_desired_angle, imuAngles.yaw + 180, 360);
        yawVelo = yaw.calculatePeriodicPosition(error, timeSure - prevTimeSure, chassis_rotation_rpm);
        #else
        yawVelo = -jyaw * JOYSTICK_SENSITIVITY_YAW_DPS / 360.0 * 60;
        #endif
        //yawVelo = 0;
        // yawVelo -= chassis_rotation_rpm;
    
        int dir = 0;
        if(yawVelo > 1){
            dir = 1;
        }else if(yawVelo < -1){
            dir = -1;
        }
        yaw.pidSpeed.feedForward = 1221 * dir + 97.4 * yawVelo;
        yaw.setSpeed(yawVelo);
    }else{
        //Off
        yaw.setPower(0);
        #ifdef USE_IMU
        yaw_desired_angle = imuAngles.yaw + 180;
        #endif
    }

    prevTimeSure = timeSure;
    timeSure = us_ticker_read();

    //PITCH
    pitch_current_angle = (pitch_zero_offset_ticks - (pitch>>ANGLE)) / TICKS_REVOLUTION * 360;
    if (drive == 'u' || drive == 'd' || (drive =='o' && (remote.rightSwitch() == Remote::SwitchState::UP || remote.rightSwitch() == Remote::SwitchState::DOWN))){
        //Regular Pitch Code
        if (pitch_desired_angle <= LOWERBOUND) {
            pitch_desired_angle = LOWERBOUND;
        }
        else if (pitch_desired_angle >= UPPERBOUND) {
            pitch_desired_angle = UPPERBOUND;
        }
    
        pitchVelo = -pitchCascade.calculatePeriodic(pitch_desired_angle - pitch_current_angle, timeSure - prevTimeSure);
        
        int dir = 0;
        if(pitchVelo > 1){
            dir = 1;
        }else if(pitchVelo < -1){
            dir = -1;
        }
    
        float pitch_current_radians = -(pitch_current_angle / 360) * 2 * M_PI;
        pitch.pidSpeed.feedForward = (cos(pitch_current_radians) * -2600) + (1221 * dir + 97.4 * pitchVelo);
        //pitch.setPosition(-int((pitch_desired_angle / 360) * TICKS_REVOLUTION - pitch_zero_offset_ticks));
        pitch.setSpeed(pitchVelo);
    }else{
        //Off
        pitch.setPower(0);
    }
}

void jetson_executor() {
    jetson_send_data.chassis_x_velocity = 0.0;
    jetson_send_data.chassis_y_velocity = 0.0;
    jetson_send_data.pitch_angle_rads = ChassisSubsystem::ticksToRadians( (pitch_zero_offset_ticks - pitch.getData(ANGLE)) );
    jetson_send_data.pitch_velocity = pitch.getData(VELOCITY) / 60.0;
    jetson_send_data.yaw_angle_rads = (imuAngles.yaw + 180.0) * (M_PI / 180.0);
    jetson_send_data.yaw_velocity = yaw.getData(VELOCITY)/60.0;
    jetson_send_feedback(bcJetson, jetson_send_data);
    
    readResult = jetson_read_values(bcJetson, jetson_received_data);
    
    if(cv_enabled){
        if(readResult > 0){
            led3 = 1;
            yaw_desired_angle = jetson_received_data.requested_yaw_rads / M_PI * 180;
            pitch_desired_angle = jetson_received_data.requested_pitch_rads / M_PI * 180;
            cv_shoot_status = jetson_received_data.shoot_status;
        }else{
            led3 = 0;
        }
    } else {
      cv_shoot_status = 0;
      led3 = 0;
    }
}

void refthread() {
    while(1) {
        mutex_test.lock();

        timeStartRef = us_ticker_read();

        //referee loop every 15ms - seems like 6ms when dc and 600us when connected
        if ((timeStart - loopTimerRef) / 1000 > 5 * OUTER_LOOP_DT_MS){ 
            loopTimerRef = timeStart;
            led2 = referee.readable();
            refereeThread(&referee);

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
            imu.get_angular_position_quat(&imuAngles);
            #else
            yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
            #endif
        }

        mutex_test.unlock();
        ThisThread::sleep_for(1ms);
    }
}

void init() {
    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();
    usbSerial.set_blocking(false);
    bcJetson.set_blocking(false);
    
    /*
    * MOTORS SETUP AND PIDS
    */
    //YAW
    yaw.setSpeedPID(569.2333, 0.988, 2.6284);
    yaw.setSpeedIntegralCap(8000);
    yaw.setSpeedOutputCap(32000);
    yaw.setPositionPID(0.98, 0, 0);
    yaw.pidPosition.dBuffer.lastY = 5;
    yaw.pidPosition.setIntegralCap(2);
    yaw.pidPosition.setOutputCap(90);
    yaw.outputCap = 16000;
    yaw.useAbsEncoder = false;
    
    //CHASSIS
    Chassis.setYawReference(&yaw, 6500); //the number of ticks of yaw considered to be robot-front
    //Common values for reference are 6500 and 2500
    Chassis.setSpeedFF_Ks(CHASSIS_FF_KICK); //feed forward "kick" for wheels, a constant multiplier of max power in the direcion of movment
    
    #ifdef USE_IMU
    imu.get_angular_position_quat(&imuAngles);
    yaw_desired_angle = imuAngles.yaw + 180;
    #else
    yaw_desired_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    yaw_current_angle = (yaw>>ANGLE) * 360.0 / TICKS_REVOLUTION;
    #endif
    
    //PITCH
    pitchCascade.setIntegralCap(2);
    pitchCascade.setOutputCap(30);
    pitch.setSpeedPID(500,0.8,0);
    pitch.setSpeedIntegralCap(2000);
    pitch.setSpeedOutputCap(32000);
    pitch.pidPosition.feedForward = 0;
    pitch.outputCap = 16000;
    pitch.useAbsEncoder = true;
    pitchCascade.dBuffer.lastY = 5;
    
    //FLYWHEELS
    LFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    RFLYWHEEL.setSpeedPID(7.1849, 0.000042634, 0);
    
    //INDEXER
    indexer.setSpeedPID(2.7, 0.001, 0);
    indexer.setSpeedIntegralCap(100);
    //Cascading PID for indexer angle position control. Surely there are better names then "sure"...
    sure.setOutputCap(133 * M2006_GEAR_RATIO);
    sure.dBuffer = 10;

    // JETSON
    jetson_send_data.chassis_x_velocity = 0.0;
    jetson_send_data.chassis_y_velocity = 0.0;
    jetson_send_data.pitch_angle_rads = 0.0;
    jetson_send_data.yaw_angle_rads = 0.0;
    jetson_send_data.pitch_velocity = 0.0;
    jetson_send_data.yaw_velocity = 0.0;
    jetson_received_data.requested_pitch_rads = 0.0;
    jetson_received_data.requested_yaw_rads = 0.0;
    jetson_received_data.shoot_status = 0;
}


#endif //TR_EMBEDDED_INFANTRY_H
