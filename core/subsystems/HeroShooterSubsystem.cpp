#include "HeroShooterSubsystem.h"
#include "util/motor/DJIMotor.h"

HeroShooterSubsystem::HeroShooterSubsystem(config cfg):
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
    }),
    feeder({
        cfg.feeder_id,
        cfg.canBus,
        M2006,
        "Feeder",
        cfg.feeder_PID
    })
{
    // initialize all other vars
    shoot = OFF;
    shootReady = true;

    barrel_heat_limit = cfg.heat_limit;
    invert_flywheel = cfg.invert;

    shooter_time = us_ticker_read();
}


void HeroShooterSubsystem::setState(ShootState shoot_state)
{
    shoot = shoot_state;
}


void HeroShooterSubsystem::periodic(int curr_heat, int heat_limit) 
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
        if (!invert_flywheel) {
            flywheelL.setSpeed(-FLYWHEEL_VELO);
            flywheelR.setSpeed(FLYWHEEL_VELO);
        }
        else {
            flywheelL.setSpeed(FLYWHEEL_VELO);
            flywheelR.setSpeed(-FLYWHEEL_VELO);
        }

        indexer.pidSpeed.feedForward = 0;
        indexer.setSpeed(0);
        shootReady = true;
        if(barrel_heat_limit < 10 || barrel_heat < barrel_heat_limit - 110) {
            shooter_time = us_ticker_read();
        }
    }
    else if (shoot == SHOOT)
    {
        if (!invert_flywheel) {
            flywheelL.setSpeed(-FLYWHEEL_VELO);
            flywheelR.setSpeed(FLYWHEEL_VELO);
        }
        else {
            flywheelL.setSpeed(FLYWHEEL_VELO);
            flywheelR.setSpeed(-FLYWHEEL_VELO);
        }

        if ((us_ticker_read() - shooter_time)/1000 < 200){
            feeder.setSpeed(7000);
        } else {
            feeder.setSpeed(0);
        }
        if ((us_ticker_read() - shooter_time)/1000 < 500){
            indexer.setSpeed(6000);
        }else if ((us_ticker_read() - shooter_time)/1000 > 500 && (us_ticker_read() - shooter_time)/1000 < 650){
            indexer.setPower(-16000);
        } else {
            indexer.setPower(0);
        }
    }
}