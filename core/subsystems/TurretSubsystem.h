#pragma once

#include "peripherals/imu/BNO055.h"

class TurretSubsystem
{
public:
    // pitch zero offset ticks
    // pitch motor
    // yaw motor
    TurretSubsystem();

    TurretSubsystem(int yaw_id, int pitch_id, BNO055_ANGULAR_POSITION_typedef* imuAngles);

    // get angle zero offsetted
    double get_pitch_angle_rads_zero_offsetted();

    double get_pitch_vel_rads_per_sec();

    double get_yaw_vel_rads_per_sec();

    void periodic();

private:
    bool configured;
};