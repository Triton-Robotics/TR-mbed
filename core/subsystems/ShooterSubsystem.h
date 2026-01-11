

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

    struct config
    {
        ShooterType type;
        int heat_limit;

        short flywheelL_id;
        short flywheelR_id;
        short indexer_id;

        PID::config flywheelL_PID;
        PID::config flywheelR_PID;
        PID::config indexer_PID_vel;
        PID::config indexer_PID_pos;

        CANHandler::CANBus canBus;
    };

    // TODO probably want explicate constructors?
    ShooterSubsystem();

    ShooterSubsystem(config configuration);

    void init(config configuration);

    ShootState getState();

    // TODO: make this nicer
    inline void setHeatLimit(int heat_limit) {barrel_heat_limit = heat_limit;};
    
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
