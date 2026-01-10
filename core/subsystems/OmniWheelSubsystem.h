#pragma once

#include "mbed.h"
#include "util/peripherals/Sensor.h"
#include <cstdint>
#include <util/motor/DJIMotor.h>
#include <util/communications/CANHandler.h>
#include "Subsystem.h"
#include "TurretSubsystem.h"

#define WHEEL_DIAMETER_METERS 0.146
#define PI 3.14159265


class OmniWheelSubsystem: public Subsystem
{
public:
    struct config
    {
        short flid;
        short frid;
        short blid;
        short brid;

        float radius;
        int power_limit;

        float max_vel = 2.92;
        float max_accel = 100;
        float feed_forward = 0.065 * INT16_MAX;
        float max_beyblade = 4.8;
        
        PID::config fl_vel_config;
        PID::config fr_vel_config;
        PID::config bl_vel_config;
        PID::config br_vel_config;
        
        CANHandler::CANBus canBus;

        IMU *imu; // We should change this to be of type IMU which will obv have all the same read function

        TurretSubsystem *yaw; // Need to expose turretsubsystem yaw, or just expose the yaw>>ANGLE
        float initial_angle = 0;
        float yawAlign = 0;
    };

    struct ChassisSpeed
    {
        float vX; // Forward m/s
        float vY; // Left m/s
        float vOmega; // Clockwise rad/s
    };

    enum ChassisMode
    {
        YAW_ORIENTED,
        BEYBLADE,
        ROBOT_ORIENTED,
        ODOM_ORIENTED,
        YAW_ALIGNED,
        OFF
    };

    struct ChassisState
    {
        ChassisSpeed vel = {0.0, 0.0, 0.0};
        ChassisMode mode = OFF;
    };

    OmniWheelSubsystem();

    OmniWheelSubsystem(config cfg);

    void setChassisState(ChassisState state);

    ChassisState getChassisState();

    /**
     * Yaw motor is a motor that controls the Turret
     * yawPhase is an initial offset of your Yaw Motor Angle
     * (basically which direction you want your Heading to be w.r.t Yaw Motor)
     *
     * @param motor your Yaw Motor reference as in `&{motor_name}`
     * @param initial_offset_ticks initial offset of your Yaw Motor Angle in ticks (try pass it as float)
     */
    void setYawReference(TurretSubsystem *_yaw, float initial_offset_ticks = 0, float _yawAlign = 0);

    /**
     * Yaw motor is a motor that controls the Turret
     * yawOdom is the world pose representation of our robot
     * (basically which direction you want your Heading to be w.r.t Yaw Motor)
     */
    bool setOdomReference();

    void periodic();

private:
    struct WheelSpeed
    {
        float fl; // rpm
        float fr; // rpm
        float bl; // rpm
        float br; // rpm
    };

    ChassisState desired_state;
    ChassisState curr_state;

    WheelSpeed desired_wheelspeed;
    WheelSpeed curr_wheelspeed;

    unsigned long prev_time;
    unsigned long dt; // delta time in ms

    int motor_power[4];

    IMU *imu;
    IMU::EulerAngles imuAngles;

    DJIMotor fl;
    DJIMotor fr;
    DJIMotor bl;
    DJIMotor br;

    // We need to establish a yaw encoder reference here for head to body conversion
    TurretSubsystem *yaw; // pointer to turret for head reference
    int yawAlign;
    float yawPhase;
    float yawOdom;
    float imuOdom;

    float radius;
    int power_limit;
    
    float FF_Ks;
    float maxVel;
    float maxAccel;
    float maxBeyblade;

    // TODO :) (idk if we need em)
    // void calculateAccelLimit();
    // void setOmniKinematics();

    // Updates curr_state and curr_wheelspeed from motors
    void getOmniState();

    // Calculates necessary wheelspeeds for the desired state
    void setDesiredWheelSpeed();

    // Calculate the ChassisSpeed in the desired frame of reference
    ChassisSpeed rotateChassisSpeed(ChassisSpeed desired_vel, float frame_angle);

    // Calculate and update the desired wheelSpeed using the rotated chassisSpeed
    void calculateWheelSpeed(ChassisSpeed chassisSpeeds);

    // Calculate the ChassisSpeed from the current WheelSpeed
    void calculateChassisSpeed();

    // Sends the power needed for the wheels to reach the desired speeds
    void sendPower();

    // Bisection
    float Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm, float chassisPowerLimit);

    // p-theory
    float p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm);
};