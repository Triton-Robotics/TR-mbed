#include "TurretSubsystem.h"
#include "us_ticker_defines.h"
#include "util/motor/DJIMotor.h"

TurretSubsystem::TurretSubsystem()
{
    configured = false;
}

TurretSubsystem::TurretSubsystem(config cfg)
{
    yaw = DJIMotor(cfg.yaw_id, cfg.yawCanBus, M3508);
    pitch = DJIMotor(cfg.pitch_id, cfg.pitchCanBus, M3508_FLYWHEEL);

    yaw.setSpeedPID(cfg.yaw_vel_PID.kp,
                    cfg.yaw_vel_PID.ki,
                    cfg.yaw_vel_PID.kd);

    yaw.setPositionPID(cfg.yaw_pos_PID.kp,
                    cfg.yaw_pos_PID.ki,
                    cfg.yaw_pos_PID.kd);
    
    pitch.setSpeedPID(cfg.pitch_vel_PID.kp,
                    cfg.pitch_vel_PID.ki,
                    cfg.pitch_vel_PID.kd);

    pitch.setPositionPID(cfg.pitch_vel_PID.kp,
                    cfg.pitch_vel_PID.ki,
                    cfg.pitch_vel_PID.kd);
    
    pitch_offset_ticks = cfg.pitch_offset_ticks;

    turretState = SLEEP;

    configured = true;
}


TurretSubsystem::TurretInfo TurretSubsystem::getState()
{
    return turret_state;
}

void TurretSubsystem::setState (TurretState des_state)
{
    turretState = des_state;
}

int TurretSubsystem::getTicks()
{
    return yaw.getData(ANGLE);
}

double TurretSubsystem::get_pitch_angle_rads_zero_offsetted()
{
    return (pitch_offset_ticks - (pitch>>ANGLE)) / TICKS_REVOLUTION * 360;
}

// Unsure if this is the best way to expose yaw, but we could do a similar thing for pitch once pitch imu
double TurretSubsystem::get_yaw_angle_rads()
{
    return imu->read().yaw;
}

double TurretSubsystem::get_pitch_vel_rads_per_sec()
{
    return pitch.getData(VELOCITY);
}

double TurretSubsystem::get_yaw_vel_rads_per_sec()
{
    return yaw.getData(VELOCITY);
}

void TurretSubsystem::set_desired_turret(float des_yaw_angle, float des_pitch_angle, float chassisRpm)
{
    des_yaw = des_yaw_angle;
    des_pitch = des_pitch_angle;
    chassis_rpm = chassisRpm;
}

void TurretSubsystem::execute_turret() 
{
    if (turretState == SLEEP)
    {
        yaw.setPower(0);
        pitch.setPower(0);
    }
    else if (turretState == AIM) 
    {
        // Yaw calc
        int dir = 0;
        if(turret_state.yaw_velo > 1){
            dir = 1;
        }else if(turret_state.yaw_velo < -1){
            dir = -1;
        }
        yaw.pidSpeed.feedForward = 1221 * dir + 97.4 * turret_state.yaw_velo;
        int des_yaw_power = yaw.calculatePositionPID(des_yaw, turret_state.yaw_angle, turret_time, chassis_rpm);
        yaw.setPower(des_yaw_power);


        // Pitch calc
        float pitch_current_radians = -(turret_state.pitch_angle / 360) * 2 * M_PI;
        
        if (des_pitch <= LOWERBOUND) {
            des_pitch = LOWERBOUND;
        }
        else if (des_pitch >= UPPERBOUND) {
            des_pitch = UPPERBOUND;
        }

        int dir_p = (-1 ? des_pitch < turret_state.pitch_angle : 1);
        pitch.pidSpeed.feedForward = (cos(pitch_current_radians) * -2600) + (1221 * dir_p + 97.4 * turret_state.pitch_velo);

        float des_pitch_power = pitch.calculatePositionPID(des_pitch, turret_state.pitch_angle, us_ticker_read() - turret_time);
        
        pitch.setPower(des_pitch_power);
    }
}

void TurretSubsystem::periodic()
{
    // TODO do we recalculate our values here?
    execute_turret();
    
    // Update turret_state here!
    turret_state.yaw_angle = get_yaw_angle_rads();
    turret_state.pitch_angle = get_pitch_angle_rads_zero_offsetted();
    turret_state.yaw_velo = get_yaw_vel_rads_per_sec();
    turret_state.pitch_velo = get_pitch_vel_rads_per_sec();

    turret_time = us_ticker_read();
}
