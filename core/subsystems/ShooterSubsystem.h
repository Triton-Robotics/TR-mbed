

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
        ShooterType type;
        int heat_limit;

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
    
    void setState(ShootState shoot_state, int curr_heat);

    void execute_shooter();

    void periodic();

private:
    // TODO Decide necessary variables for state and control
    bool configured;

    unsigned long shooter_time;

    DJIMotor flywheelL, flywheelR, indexer;

    ShootState shoot;
    ShooterType shooter_type;
    
    int barrel_heat;
    int barrel_heat_limit;
    bool shootReady;

    int shootTargetPosition;

    // If ready to shoot, set the desired position for indexer to move
    void setTargetPos();
};
