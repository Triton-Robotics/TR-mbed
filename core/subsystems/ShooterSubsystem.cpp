#include "ShooterSubsystem.h"
#include "us_ticker_defines.h"
#include "util/motor/DJIMotor.h"

// TODO: add a way to invert which flywheel is -ve
ShooterSubsystem::ShooterSubsystem(config cfg):
    flywheelL({
        cfg.flywheelL_id,
        cfg.canBus,
        M3508,
        "Left Flywheel",
        cfg.flywheelL_PID
    }),
    flywheelR({
        cfg.flywheelR_id,
        cfg.canBus,
        M3508,
        "Right Flywheel",
        cfg.flywheelR_PID
    }),
    indexer({
        cfg.indexer_id,
        cfg.canBus,
        M3508,
        "Indexer",
        cfg.indexer_PID_vel,
        cfg.indexer_PID_pos
    })
{
    // initialize all other vars
    shoot = OFF;
    shootReady = true;

    barrel_heat_limit = cfg.heat_limit;

    shooter_type = cfg.type;
    shooter_time = us_ticker_read();
    // printf("shooterinit\n");
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

void ShooterSubsystem::periodic(int curr_heat, int heat_limit) 
{
    barrel_heat = curr_heat;
    barrel_heat_limit = heat_limit;

    if (shoot == OFF) 
    {
        flywheelL.setPower(0);
        flywheelR.setPower(0);
        indexer.setPower(0);
        shootReady = true;
    }
    else if (shoot == FLYWHEEL)
    {
        flywheelL.setSpeed(-FLYWHEEL_VELO);
        flywheelR.setSpeed(FLYWHEEL_VELO);

        indexer.pidSpeed.feedForward = 0;
        indexer.setSpeed(0);
        shootReady = true;
    }
    else if (shoot == SHOOT && shooter_type == BURST)
    {
        flywheelL.setSpeed(-FLYWHEEL_VELO);
        flywheelR.setSpeed(FLYWHEEL_VELO);

        // Set target position
        if (shootReady && (abs(flywheelR>>VELOCITY) > (FLYWHEEL_VELO - 500) && abs(flywheelL>>VELOCITY) > (FLYWHEEL_VELO - 500))) 
        {
            // heat limit
            if(barrel_heat_limit < 10 || barrel_heat < barrel_heat_limit - 40) {
                shootReady = false; // Cant shoot twice immediately

                shootTargetPosition = (8192 * M2006_GEAR_RATIO / 9 * NUM_BALLS_SHOT) + (indexer>>MULTITURNANGLE);
            }
        }

        // TODO fix to 1 degree of error allowed (this is more than 1 degree)
        if (abs((indexer >> MULTITURNANGLE) - shootTargetPosition) <= 819)
        {
            shoot = FLYWHEEL; // Move to neutral state after shooting
        }
        else
        {
            indexer.pidSpeed.feedForward = (indexer >> VELOCITY) / 4788 * 630;
            indexer.setPosition(shootTargetPosition);
        }
    }
    else if (shoot == SHOOT && shooter_type == AUTO) 
    {
        if(barrel_heat_limit < 10 || barrel_heat < barrel_heat_limit - 30) {
            indexer.setSpeed(-5 * 16 * M2006_GEAR_RATIO);
        }
        else {
            indexer.setSpeed(0);
        }
    }
    else if (shoot == SHOOT && shooter_type == HERO) 
    {
        // TODO
    }

    shooter_time = us_ticker_read();
}