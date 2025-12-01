#include "ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem()
{
    configured = false;
}

ShooterSubsystem::ShooterSubsystem(config configuration)
{
    // TODO populate all necessary shooting objects
    flywheelL = DJIMotor(configuration.flywheelL_id, CAN_BUS_TYPE, M3508);
    flywheelR = DJIMotor(configuration.flywheelR_id, CAN_BUS_TYPE, M3508);
    indexer = DJIMotor(configuration.indexer_id, CAN_BUS_TYPE, M2006);

    flywheelL.setSpeedPID(configuration.flywheelL_PID.kp,
                          configuration.flywheelL_PID.ki,
                          configuration.flywheelL_PID.kd);

    flywheelR.setSpeedPID(configuration.flywheelR_PID.kp,
                          configuration.flywheelR_PID.ki,
                          configuration.flywheelR_PID.kd);

    indexer.setSpeedPID(configuration.indexer_PID.kp,
                        configuration.indexer_PID.ki,
                        configuration.indexer_PID.kd);

    // initialize all other vars
    configured = true;
    shoot = OFF;
    shooter_time = us_ticker_read();
    shooter_type = configuration.shoot_type;
}

void ShooterSubsystem::setState(ShootState shoot_state)
{
    shoot = shoot_state;
}

void ShooterSubsystem::execute_shooter() 
{
    if (shoot == OFF) 
    {
        flywheelL.setPower(0);
        flywheelR.setPower(0);
        indexer.setPower(0);
    }
    else if (shoot == FLYWHEEL)
    {
        flywheelL.setSpeed(-FLYWHEEL_VELO);
        flywheelR.setSpeed(FLYWHEEL_VELO);
        indexer.setPower(0);
    }
    else if (shoot == SHOOT)
    {
        // TODO: shoot logic here
        if (shooter_type == BURST)
        shootTargetPosition = (8192 * M2006_GEAR_RATIO / 9 * NUM_BALLS_SHOT) + (indexer >> MULTITURNANGLE);
    }
}

void ShooterSubsystem::periodic() 
{
    unsigned long curr_time = us_ticker_read();
    // TODO Update all dynamic vars: time, shootready, shot (how to get remote data to update these?)
    // Logic to shoot goes here / in a function called here: execute_flywheels, execute_indexer
    execute_shooter();
    shooter_time = curr_time;
}