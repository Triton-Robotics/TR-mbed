#pragma once

#include "util/algorithms/general_functions.h"
#include "util/peripherals/imu/BNO055.h"
#include <util/communications/CANHandler.h>
#include <util/motor/DJIMotor.h>
#include "Subsystem.h"


// TODO add to cfg
constexpr float LOWERBOUND = -35.0;
constexpr float UPPERBOUND = 40.0;

// enums for state
enum TurretState {SLEEP, AIM};

class TurretSubsystem : public Subsystem
{
public:
    struct PID_config 
    {
        float kp;
        float ki;
        float kd;
    };

    struct config
    {
        short yaw_id;
        motorType yaw_type = motorType::GM6020;
        short pitch_id;
        motorType pitch_type = motorType::GM6020;
        
        int pitch_offset_ticks;

        PID::config yaw_vel_PID;
        PID::config yaw_pos_PID;

        PID::config pitch_vel_PID;
        PID::config pitch_pos_PID;

        CANHandler::CANBus yawCanBus;
        CANHandler::CANBus pitchCanBus;
        int forward;

        IMU &imu;
    };

    struct TurretInfo
    {
        float yaw_angle;
        float yaw_velo;
        float pitch_angle;
        float pitch_velo;
    };

    TurretSubsystem(config cfg);

    TurretInfo getState();

    void setState (TurretState state);

    void set_desired_turret(float des_yaw_angle, float des_pitch_angle, float chassisRpm);

    void execute_turret();

    void periodic();

    int getTicks();
    
    DJIMotor yaw; 
private:

    DJIMotor pitch;

    TurretState turretState;
    TurretInfo turret_state;

    IMU &imu;
    IMU::EulerAngles imuAngles;

    bool configured;
    const int forward_;

    int pitch_offset_ticks;

    unsigned long turret_time;

    float des_yaw, des_pitch, chassis_rpm;

    // get angle zero offsetted
    float get_pitch_angle_degs_zero_offsetted();

    float get_yaw_angle_degs();

    float get_pitch_vel_rads_per_sec();

    float get_yaw_vel_rads_per_sec();
};