

#pragma once

#include <util/algorithms/PID.h>
#include <util/communications/CANHandler.h>
#include <util/motor/DJIMotor.h>
#include "Subsystem.h"

// Constants
constexpr int NUM_BALLS_SHOT = 3;
constexpr int FLYWHEEL_VELO = 5500;

// enum for states
// TODO simplify enums
enum ShootState {OFF, FLYWHEEL, SHOOT};

// struct for config
class ShooterSubsystem : public Subsystem
{
public:
    // TODO add compiler flag -Wmissing-field-initializers

    enum ShooterType {BURST, HERO, AUTO};
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

    void init(config configuration);

    ShootState getState();
    
    void setState(ShootState shoot_state);

    void execute_shooter();

    void periodic();

private:
    // TODO Decide necessary variables for state and control
    bool configured;

    unsigned long shooter_time;

    DJIMotor flywheelL, flywheelR, indexer;

    ShootState shoot;
    ShooterType shooter_type;

    int shootTargetPosition;
};
