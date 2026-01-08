#pragma once

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
        short pitch_id;
        
        int pitch_offset_ticks;

        PID::config yaw_vel_PID;
        PID::config yaw_pos_PID;

        PID::config pitch_vel_PID;
        PID::config pitch_pos_PID;

        CANHandler::CANBus yawCanBus;
        CANHandler::CANBus pitchCanBus;

        IMU *imu;
    };

    struct TurretInfo
    {
        float yaw_angle;
        float yaw_velo;
        float pitch_angle;
        float pitch_velo;
    };

    TurretSubsystem();

    TurretSubsystem(config cfg);

    TurretInfo getState();

    void setState (TurretState state);

    void set_desired_turret(float des_yaw_angle, float des_pitch_angle, float chassisRpm);

    void execute_turret();

    void periodic();

    int getTicks();

private:
    bool configured;

    DJIMotor yaw, pitch;

    TurretState turretState;
    TurretInfo turret_state;

    IMU *imu;
    IMU::EulerAngles imuAngles;

    int pitch_offset_ticks;

    unsigned long turret_time;

    float des_yaw, des_pitch, chassis_rpm;

    // get angle zero offsetted
    double get_pitch_angle_degs_zero_offsetted();

    double get_yaw_angle_degs();

    double get_pitch_vel_rads_per_sec();

    double get_yaw_vel_rads_per_sec();
};