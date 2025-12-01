

#pragma once

#include <algorithms/PID.h>
#include <communications/CANHandler.h>
#include <motor/DJIMotor.h>

// Constants
constexpr int NUM_BALLS_SHOT = 3;
constexpr int FLYWHEEL_VELO = 5500;

// enum for states'
// TODO simplify enums
enum ShootState {OFF, FLYWHEEL, SHOOT};
enum ShooterType {BURST, HERO, AUTO};

// struct for config
class ShooterSubsystem
{
public:
    // TODO add compiler flag -Wmissing-field-initializers
    struct PID_config 
    {
        float kp;
        float ki;
        float kd;
    };

    struct config
    {
        int flywheelL_id;
        int flywheelR_id;
        int indexer_id;
        PID_config flywheelL_PID;
        PID_config flywheelR_PID;
        PID_config indexer_PID;
        CANHandler::CANBus canBus;
    };

    // TODO probably want explicate constructors?
    ShooterSubsystem();

    ShooterSubsystem(config configuration);

    void setState(ShootState shoot_state);

    void execute_shooter();

    void periodic();

private:
    bool configured;

    unsigned long shooter_time;

    DJIMotor flywheelL, flywheelR, indexer;

    ShootState shoot;

    ShooterType shooter_type;

    int shootTargetPosition;
    
};
