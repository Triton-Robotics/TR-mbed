#include "ShooterSubsystem.h"
#include "util/motor/DJIMotor.h"
#include <us_ticker_api.h>

ShooterSubsystem::ShooterSubsystem(config cfg):
    flywheelL({
        cfg.flywheelL_id,
        cfg.canBus,
        M3508_FLYWHEEL,
        "Left Flywheel",
        cfg.flywheelL_PID
    }),
    flywheelR({
        cfg.flywheelR_id,
        cfg.canBus,
        M3508_FLYWHEEL,
        "Right Flywheel",
        cfg.flywheelR_PID
    }),
    indexer({
        cfg.indexer_id,
        cfg.canBus,
        M2006, // TODO REMOVE PREDEFINTION, MAKE IT A CONFIG!
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
    invert_flywheel = cfg.invert;
    shooter_time = us_ticker_read();
    shooter_start_timer = 0;
}

void ShooterSubsystem::setFlywheels() {
    if (!invert_flywheel) {
            flywheelL.setSpeed(-FLYWHEEL_VELO);
            flywheelR.setSpeed(FLYWHEEL_VELO);
        }
    else {
            flywheelL.setSpeed(FLYWHEEL_VELO);
            flywheelR.setSpeed(-FLYWHEEL_VELO);
        }
}

void ShooterSubsystem::reverseFlywheels() {
    if (!invert_flywheel) {
            flywheelL.setSpeed(FLYWHEEL_VELO);
            flywheelR.setSpeed(-FLYWHEEL_VELO);
        }
    else {
            flywheelL.setSpeed(-FLYWHEEL_VELO);
            flywheelR.setSpeed(FLYWHEEL_VELO);
        }
}


void ShooterSubsystem::setState(ShootState shoot_state)
{
    shoot = shoot_state;
}

void ShooterSubsystem::periodic(int curr_heat, int heat_limit) 
{
    // These first 3 if statements do dedicated unjamming at start up
    if (shooter_start_timer == 0) {
        shooter_start_timer = us_ticker_read();
    }
    // My code is a lil funny but this statement runs after the 3rd if, it just "vomits" the ball out if stuck; I thought writing like this would make it slightly more optimized
    if ((us_ticker_read() - shooter_start_timer) > 4000000 && (us_ticker_read() - shooter_start_timer) < 5000000) { // For first 4 seconds, reverse flywheels to prevent jamming
        setFlywheels();
        return;
    }
    // For first 4 seconds, the flywheels reverse so that the ball backs up. Be aware that the head should face down.
    // If the head is facing up the ball isn't vomited out, and could re-jam if you turn the head down w/o shooting first
    else if ((us_ticker_read() - shooter_start_timer) < 4000000) { 
        reverseFlywheels();
        return;
    }

    barrel_heat = curr_heat;
    barrel_heat_limit = heat_limit;

    if (shoot == OFF) 
    {
        flywheelL.setSpeed(0);
        flywheelR.setSpeed(0);
        // flywheelL.setPower(0);
        // flywheelR.setPower(0);
        
        indexer.setPower(0);
        shootReady = true;
        shooter_time = 0;
    }
    else if (shoot == FLYWHEEL)
    {
        setFlywheels();

        indexer.pidSpeed.feedForward = 0;
        indexer.setSpeed(0);
        shootReady = true;
    }
    else if (shoot == SHOOT && shooter_type == BURST)
    {
        setFlywheels();
        
        if (abs(flywheelR >> VELOCITY) < abs(FLYWHEEL_VELO * 0.75) || abs(flywheelL >> VELOCITY) < abs(FLYWHEEL_VELO * 0.75)) {
            return;
        }

        // Set target position
        if (shootReady && (abs(flywheelR>>VELOCITY) > (FLYWHEEL_VELO - 500) && abs(flywheelL>>VELOCITY) > (FLYWHEEL_VELO - 500))) 
        {
            // heat limit
            if(barrel_heat_limit < 10 || barrel_heat < barrel_heat_limit - 40) {
                shootReady = false; // Cant shoot twice immediately

                shootTargetPosition = (8192 * M2006_GEAR_RATIO / 9 * NUM_BALLS_SHOT) + (indexer>>MULTITURNANGLE);
                backfeedPosition = (indexer>>MULTITURNANGLE) - (8192 * M2006_GEAR_RATIO / 9 * NUM_BALLS_SHOT);
            }
        }

        // TODO fix to 1 degree of error allowed (this is more than 1 degree)
        if (abs((indexer >> MULTITURNANGLE) - shootTargetPosition) <= 819)
        {
            shoot = FLYWHEEL; // Move to neutral state after shooting
        }
        else
        {

            // Preventative Backfeeding logic
            
            if ((abs(float(indexer>>TORQUE) / 5596) > 1.65 && (abs((indexer >> MULTITURNANGLE) - shootTargetPosition) > int(8192/360)))) {
                printf("%.2f, %d BACKFEED TIME\n", abs(float(indexer>>TORQUE) / 5596), abs((indexer >> MULTITURNANGLE) - shootTargetPosition));
                indexer.pidSpeed.feedForward = (-(indexer>>VELOCITY) / 4788.0) * 630 * 6; // todo what are these magic numbers?
                //indexer.setSpeed(-60 * M2006_GEAR_RATIO);
                indexer.setPosition(backfeedPosition);
                
                // Reversing flywheel direction to unjam 
                reverseFlywheels();

                // if (!jammed) {
                //     // First time entering — record the start time
                //     jamCurrTime = us_ticker_read();
                //     jammed = 1;
                // }

                // // Check if 250ms have elapsed since we entered the jam state
                // if (us_ticker_read() - jamCurrTime > 250000) {
                //     jammed = 0;
                // }


            }

            indexer.pidSpeed.feedForward = (indexer >> VELOCITY) / 4788 * 630;
            indexer.setPosition(shootTargetPosition);
        }
    }
    else if (shoot == SHOOT && shooter_type == AUTO) 
    {
        if (!invert_flywheel) {
            flywheelL.setSpeed(-FLYWHEEL_VELO);
            flywheelR.setSpeed(FLYWHEEL_VELO);
        }
        else {
            flywheelL.setSpeed(FLYWHEEL_VELO);
            flywheelR.setSpeed(-FLYWHEEL_VELO);
        }

        if(barrel_heat_limit < 10 || barrel_heat < barrel_heat_limit - 30) {
            indexer.setSpeed(-5 * 16 * M2006_GEAR_RATIO);
        }
        else {
            indexer.setSpeed(0);
        }
    }
    else if (shoot == JAM || jammed) { // Dedicated Unjamming state
        if (!jammed) {
            // First time entering — record the start time
            printf("BACKFEED TIME\n");
            jamCurrTime = us_ticker_read();
            jammed = 1;

            // Immediately reverse flywheels
            reverseFlywheels();
        }
        if (us_ticker_read() - jamCurrTime < 150000) { //Reverse Indexer for 150ms 
                indexer.pidSpeed.feedForward = (-(indexer>>VELOCITY) / 4788) * 630 * 6; 
                //indexer.setSpeed(-60 * M2006_GEAR_RATIO);
                indexer.setPosition(backfeedPosition);
            }
                
        else if (us_ticker_read() - jamCurrTime > 150000) { // After 150ms, reset flywheel
            setFlywheels();
                }
        if (us_ticker_read() - jamCurrTime > 200000) { // After 200ms, go back to normal
            jammed = 0;
        } 
    }

    shooter_time = us_ticker_read();
}