#pragma once

#include "util/peripherals/imu/BNO055.h"
#include <util/communications/CANHandler.h>
#include <util/motor/DJIMotor.h>
#include "Subsystem.h"

// enums for state
enum TurretState {SLEEP, AIM};

class TurretSubsystem : public Subsystem
{
public:
    // pitch zero offset ticks
    // pitch motor
    // yaw motor
    struct PID_config 
    {
        float kp;
        float ki;
        float kd;
    };

    struct config
    {
        int yaw_id;
        int pitch_id;
        int pitch_offset_ticks;
        PID_config yaw_vel_PID;
        PID_config yaw_pos_PID;
        PID_config pitch_vel_PID;
        PID_config pitch_pos_PID;
        CANHandler::CANBus yawCanBus;
        CANHandler::CANBus pitchCanBus;
        BNO055_ANGULAR_POSITION_typedef* imuAngles;
    };

    struct TurretInfo
    {
        float yaw_angle;
        float yaw_velo;
        float pitch_angle;
        float pitch_velo;
    };

    TurretSubsystem();

    TurretSubsystem(config configuration);

    void init(config configuration);

    TurretInfo getState();

    void setState (TurretState state);

    void set_desired_turret(float des_yaw_angle, float des_pitch_angle, float chassisRpm);

    void execute_turret();

    void periodic();

    // get angle zero offsetted
    double get_pitch_angle_rads_zero_offsetted();

    double get_pitch_vel_rads_per_sec();

    double get_yaw_vel_rads_per_sec();

private:
    bool configured;

    DJIMotor yaw, pitch;

    TurretState turretState;

    TurretInfo turret_state;

    int pitch_offset_ticks;

    unsigned long turret_time;

    float des_yaw, des_pitch, chassis_rpm;
};