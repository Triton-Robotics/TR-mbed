#pragma once

#include "mbed.h"
#include "util/peripherals/Sensor.h"
#include <util/motor/DJIMotor.h>
#include <util/communications/CANHandler.h>
#include "Subsystem.h"
#include "TurretSubsystem.h"


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
        float power_limit;
        
        PID::config fl_vel_config;
        PID::config fr_vel_config;
        PID::config bl_vel_config;
        PID::config br_vel_config;
        
        CANHandler::CANBus canBus;

        Sensor *imu; // We should change this to be of type IMU which will obv have all the same read function

        TurretSubsystem *turret;
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
    void setYawReference(TurretSubsystem *_turret, float initial_offset_ticks = 0, float _yawAlign = 0);

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
    ChassisState prev_state;

    WheelSpeed desired_wheelspeed;
    WheelSpeed curr_wheelspeed;
    WheelSpeed prev_wheelspeed;

    float FF_Ks; // TODO init this var
    unsigned long prev_time;
    unsigned long dt; // delta time in ms

    int motor_power[4];

    Sensor *imu;
    BNO055_ANGULAR_POSITION_typedef imuAngles;

    DJIMotor fl;
    DJIMotor fr;
    DJIMotor bl;
    DJIMotor br;

    TurretSubsystem *turret; // pointer to turret for head reference
    int yawAlign;
    float yawPhase;
    float yawOdom;
    float imuOdom;

    float radius;
    float power_limit;
    float max_vel;
    float maxAccel; // TODO init this

    void calculateAccelLimit();
    void setOmniKinematics();

    // Update wheel speed from the current rpm of the motors
    void getWheelSpeed();

    // Updates chassis velocity from curr_wheelspeed
    void getOmniState();

    // Calculates necessary wheelspeeds for the desired state
    void setDesiredWheelSpeed();

    // Calculate the ChassisSpeed in the desired frame of reference
    ChassisSpeed rotateChassisSpeed(ChassisSpeed desired_vel, float frame_angle);

    // Calculate and update the desired wheelSpeed using the rotated chassisSpeed
    void calculateWheelSpeed(ChassisSpeed chassisSpeeds);

    // Sends the power needed for the wheels to reach the desired speeds
    void sendPower();

    // Bisection
    float Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm, float chassisPowerLimit);

    // p-theory
    float p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm);
};