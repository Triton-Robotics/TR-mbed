

#pragma once

#include <algorithms/PID.h>
// enum for states

// struct for config
class ShooterSubsystem
{
public:
    // TODO add compiler flag -Wmissing-field-initializers
    struct config
    {
        int flywheelL_id;
        int flywheelR_id;
        int indexer_id;
        PID flywheelL_PID;
        PID flywheelR_PID;
        PID indexer_PID;
    };

    // TODO probably want explicate constructors?
    ShooterSubsystem();

    void configure(int a, int b);

private:
    bool configured;
};
