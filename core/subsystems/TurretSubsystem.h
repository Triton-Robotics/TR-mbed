#pragma once

#include "util/algorithms/general_functions.h"
#include "util/peripherals/imu/BNO055.h"
#include <util/communications/CANHandler.h>
#include <util/motor/DJIMotor.h>
#include "Subsystem.h"
#include <cmath>


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
        
        int pitch_offset_ticks; // TODO: Change this (to deg) when IMU on pitch! (also post 25' inf)

        PID::config yaw_vel_PID;
        PID::config yaw_pos_PID;

        PID::config pitch_vel_PID;
        PID::config pitch_pos_PID;

        const float yaw_static_friction = 1221; // We multiply it by dir
        const float yaw_kinetic_friction = 97.4; // We multiply this by yawvelo
        
        const float pitch_gravity_feedforward = -2600; // We multiply this by cos(angle)
        const float pitch_static_friction = 1221; // We multiply it by dir
        const float pitch_kinetic_friction = 97.4; // We multiply this by pitchvelo
        
        CANHandler::CANBus yawCanBus;
        CANHandler::CANBus pitchCanBus;
        int forward;

        float gear_ratio = 1;

        const float pitch_lower_bound;
        const float pitch_upper_bound;
    };

    struct TurretInfo
    {
        float yaw_angle_degs;
        float yaw_velo_rad_s;
        float pitch_angle_degs;
        float pitch_velo_rad_s;
        TurretState turret_mode = SLEEP;
    };

    TurretSubsystem(config cfg, IMU &imu);

    TurretInfo getState();

    void setState(TurretInfo state);

    // void set_desired_turret(float des_yaw_angle, float des_pitch_angle);

    void periodic(float chasssisRpm);

    int getTicks(); // only thing chassis should have is this basically, or a pointer to turret
    
    DJIMotor yaw; // TODO fix chassissubsystem and put this into private
private:

    DJIMotor pitch;

    TurretInfo turret_state;

    IMU &imu_;
    IMU::EulerAngles imuAngles;

    bool configured;
    const int forward_;
    const float pitch_lowerbound, pitch_upperbound;
    const float gear_ratio;

    const float yaw_static_friction;
    const float yaw_kinetic_friction;
    const float pitch_static_friction;
    const float pitch_kinetic_friction;
    const float pitch_gravity_feedforward;

    int pitch_offset_ticks;

    unsigned long turret_time;

    float des_yaw, des_pitch, chassis_rpm;

    // get angle zero offsetted
    float get_pitch_angle_degs_zero_offsetted();

    float get_yaw_angle_degs();

    float get_pitch_angle_degs();

    float get_pitch_vel_rads_per_sec();

    float get_yaw_vel_rads_per_sec();
};