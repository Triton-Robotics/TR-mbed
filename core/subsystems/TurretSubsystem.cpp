#include "TurretSubsystem.h"
#include "us_ticker_defines.h"
#include "util/motor/DJIMotor.h"

TurretSubsystem::TurretSubsystem(config cfg):
    yaw({
        cfg.yaw_id,
            cfg.yawCanBus,
            cfg.yaw_type,
            "Yaw",
            cfg.yaw_vel_PID,
            cfg.yaw_pos_PID
    }), 
    pitch({
        cfg.pitch_id,
            cfg.pitchCanBus,
            cfg.pitch_type,
            "Pitch",
            cfg.pitch_vel_PID,
            cfg.pitch_pos_PID
    }),
    imu(cfg.imu),
    forward_(cfg.forward),
    pitch_lowerbound(cfg.pitch_lower_bound),
    pitch_upperbound(cfg.pitch_upper_bound),
    yaw_static_friction(cfg.yaw_static_friction),
    yaw_kinetic_friction(cfg.yaw_kinetic_friction),
    pitch_static_friction(cfg.pitch_static_friction),
    pitch_kinetic_friction(cfg.pitch_kinetic_friction),
    pitch_gravity_feedforward(cfg.pitch_gravity_feedforward)
{
    pitch_offset_ticks = cfg.pitch_offset_ticks;
    pitch.pidPosition.dBuffer.lastY = 5;

    imuAngles = imu.getImuAngles();

    turret_state.turret_mode = SLEEP;
    configured = true;

    turret_time = us_ticker_read();
}


TurretSubsystem::TurretInfo TurretSubsystem::getState()
{
    return turret_state;
}

void TurretSubsystem::setState (TurretInfo des_state)
{
    turret_state.turret_mode = des_state.turret_mode;
    des_yaw = des_state.yaw_angle_degs;
    des_pitch = des_state.pitch_angle_degs;
}

// void TurretSubsystem::set_desired_turret(float des_yaw_angle, float des_pitch_angle)
// {
//     des_yaw = des_yaw_angle;
//     des_pitch = des_pitch_angle;
// }

int TurretSubsystem::getTicks()
{
    return yaw.getData(ANGLE);
}

float TurretSubsystem::get_pitch_angle_degs_zero_offsetted()
{
    // return (pitch_offset_ticks - (pitch>>ANGLE)) / TICKS_REVOLUTION * 360;
    // return forward_ * ((float)((pitch>>ANGLE) - pitch_offset_ticks) / TICKS_REVOLUTION) * 360;
    float pitch_angle = get_pitch_angle_degs();
    return forward_ * pitch_angle;
}

// Unsure if this is the best way to expose yaw, but we could do a similar thing for pitch once pitch imu
float TurretSubsystem::get_yaw_angle_degs()
{
    return imuAngles.yaw;
}

// Unsure if this is the best way to expose pitch
float TurretSubsystem::get_pitch_angle_degs()
{
    return imuAngles.pitch;
}

float TurretSubsystem::get_pitch_vel_rads_per_sec()
{
    return forward_ * pitch.getData(VELOCITY);
}

float TurretSubsystem::get_yaw_vel_rads_per_sec()
{
    if (yaw.type == M3508) {
        float velo_rpm = (float)(yaw.getData(VELOCITY)) / M3508_GEAR_RATIO;
        return velo_rpm * 2 * PI / 60;
    }
    else {
        return yaw.getData(VELOCITY);
    }
}

void TurretSubsystem::periodic(float chassisRpm)
{
    // Update turret_state here!
    chassis_rpm = chassisRpm;
    imuAngles = imu.getImuAngles();
    turret_state.yaw_angle_degs = get_yaw_angle_degs();
    turret_state.pitch_angle_degs = get_pitch_angle_degs_zero_offsetted();
    turret_state.yaw_velo_rad_s = get_yaw_vel_rads_per_sec();
    turret_state.pitch_velo_rad_s = get_pitch_vel_rads_per_sec();

    // Set desired values for pitch and yaw
    if (turret_state.turret_mode == SLEEP)
    {
        yaw.setPower(0);
        pitch.setPower(0);
    }
    else if (turret_state.turret_mode == AIM) 
    {
        // Yaw calc
        int dir = 0;
        if(turret_state.yaw_velo_rad_s > 1){
            dir = 1;
        }else if(turret_state.yaw_velo_rad_s < -1){
            dir = -1;
        }
        yaw.pidSpeed.feedForward = yaw_static_friction * dir + yaw_kinetic_friction * turret_state.yaw_velo_rad_s;

        float deltaYaw = calculateDeltaYaw(turret_state.yaw_angle_degs, des_yaw);
        yaw.pidPosition.feedForward = -chassis_rpm; // TODO: MAKE THIS RAD/S AND USE THE KINETIC FRICTION COEFF HERE!!!!
        int des_yaw_power = yaw.calculatePeriodicPosition(deltaYaw, turret_time);
        yaw.setPower(des_yaw_power);


        // Pitch calc
        float pitch_current_radians = forward_ * (turret_state.pitch_angle_degs / 360) * 2 * M_PI;
        
        if (des_pitch <= pitch_lowerbound) {
            des_pitch = pitch_lowerbound;
        }
        else if (des_pitch >= pitch_upperbound) {
            des_pitch = pitch_upperbound;
        }

        int dir_p = forward_ * (des_pitch < turret_state.pitch_angle_degs ? -1 : 1);
        pitch.pidSpeed.feedForward = (cos(pitch_current_radians) * pitch_gravity_feedforward) + (pitch_static_friction * dir_p + pitch_kinetic_friction * turret_state.pitch_velo_rad_s);
        float des_pitch_power = pitch.calculatePositionPID(forward_ * des_pitch, forward_ * turret_state.pitch_angle_degs, us_ticker_read() - turret_time);
        
        // printf("pp %.2f | %.2f\n", des_pitch_power, -turret_state.pitch_angle);
        pitch.setPower(des_pitch_power);
    }

    turret_time = us_ticker_read();
}
