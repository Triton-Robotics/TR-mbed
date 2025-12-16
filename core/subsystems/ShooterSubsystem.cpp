#include "ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem()
{
    configured = false;
}

ShooterSubsystem::ShooterSubsystem(config configuration)
{
    // TODO populate all necessary shooting objects
    flywheelL = DJIMotor(configuration.flywheelL_id, configuration.canBus, M3508);
    flywheelR = DJIMotor(configuration.flywheelR_id, configuration.canBus, M3508);
    indexer = DJIMotor(configuration.indexer_id, configuration.canBus, M2006);

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
    shooter_type = BURST;
    shooter_time = us_ticker_read();
}

void ShooterSubsystem::init(config configuration)
{
    if (configured == false)
    {
        flywheelL = DJIMotor(configuration.flywheelL_id, configuration.canBus, M3508);
        flywheelR = DJIMotor(configuration.flywheelR_id, configuration.canBus, M3508);
        indexer = DJIMotor(configuration.indexer_id, configuration.canBus, M2006);

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
        shooter_type = BURST;
        shooter_time = us_ticker_read();
    }
}


void ShooterSubsystem::setState(ShootState shoot_state)
{
    shoot = shoot_state;
}

// TODO: figure out what we want to return?
ShootState ShooterSubsystem::getState()
{
    return shoot;
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

        indexer.pidSpeed.feedForward = 0;
        indexer.setSpeed(0);
    }
    else if (shoot == SHOOT)
    {
        if (shooter_type == BURST)
        {
            shootTargetPosition = (8192 * M2006_GEAR_RATIO / 9 * NUM_BALLS_SHOT) + (indexer >> MULTITURNANGLE);

            // TODO fix to 1 degree of error allowed (this is more than 1 degree)
            if (abs((indexer >> MULTITURNANGLE) - shootTargetPosition) <= 819)
            {
                shoot = FLYWHEEL;
            }
            else
            {
                indexer.pidSpeed.feedForward = (indexer >> VALUE) / 4788 * 630;
                indexer.setPosition(shootTargetPosition);
            }
        }
        else if (shooter_type = AUTO) 
        {
        }
        else if (shooter_type == HERO) 
        {
        }
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