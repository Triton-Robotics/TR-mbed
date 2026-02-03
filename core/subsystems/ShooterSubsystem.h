

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

    ShooterSubsystem(config configuration);

    void init(config configuration);

    ShootState getState();
    
    void setState(ShootState shoot_state);

    void periodic(int curr_heat, int heat_limit);

private:
    unsigned long shooter_time;

    DJIMotor flywheelL, flywheelR, indexer;

    ShootState shoot;
    ShooterType shooter_type;
    
    int barrel_heat;
    int barrel_heat_limit;
    bool shootReady;

    int shootTargetPosition;
};
