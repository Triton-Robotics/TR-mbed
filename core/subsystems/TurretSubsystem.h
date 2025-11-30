#pragma once

class TurretSubsystem
{
public:
    // pitch zero offset ticks
    // pitch motor
    // yaw motor
    TurretSubsystem();

    // get angle zero offsetted
    double get_pitch_angle_rads_zero_offsetted();

    double get_pitch_vel_rads_per_sec();

    double get_yaw_vel_rads_per_sec();

    void configure(int a, int b);

private:
    bool configured;
};