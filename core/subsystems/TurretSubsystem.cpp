#include "TurretSubsystem.h"
#include "us_ticker_defines.h"
#include "util/motor/DJIMotor.h"

TurretSubsystem::TurretSubsystem(config cfg, IMU &imu):
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
    imu_(imu),
    forward_(cfg.forward),
    pitch_lowerbound(cfg.pitch_lower_bound),
    pitch_upperbound(cfg.pitch_upper_bound),
    gear_ratio(cfg.gear_ratio),
    yaw_static_friction(cfg.yaw_static_friction),
    yaw_kinetic_friction(cfg.yaw_kinetic_friction),
    pitch_static_friction(cfg.pitch_static_friction),
    pitch_kinetic_friction(cfg.pitch_kinetic_friction),
    pitch_gravity_feedforward(cfg.pitch_gravity_feedforward)
{
    pitch_offset_ticks = cfg.pitch_offset_ticks; // TBD remove with pitch imu
    pitch.pidPosition.dBuffer.lastY = 5;

    imuAngles = imu_.getImuAngles();

    turret_state.turret_mode = SLEEP;
    configured = true;

    turret_time = us_ticker_read();
}


TurretSubsystem::TurretInfo TurretSubsystem::getState()
{
    return turret_state;
}

void TurretSubsystem::setState(TurretInfo des_state)
{
    turret_state.turret_mode = des_state.turret_mode;
    des_yaw = des_state.yaw_angle_degs;
    des_pitch = des_state.pitch_angle_degs;
}

int TurretSubsystem::getTicks()
{
    return yaw.getData(ANGLE);
}

float TurretSubsystem::get_pitch_angle_degs_zero_offsetted()
{
    float pitch_angle = get_pitch_angle_degs();
    return forward_ * pitch_angle;
}

float TurretSubsystem::get_yaw_angle_degs()
{
    return imuAngles.yaw;
}

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
    return yaw.getData(VELOCITY) * gear_ratio;
}

void TurretSubsystem::periodic(float chassisRpm)
{
    // Update turret_state here!
    chassis_rpm = chassisRpm;
    imuAngles = imu_.getImuAngles();
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
        float deltaYaw = calculateDeltaYaw(turret_state.yaw_angle_degs, des_yaw);
        
        if (std::isnan(deltaYaw) || std::isnan(turret_state.pitch_angle_degs)) { // On conn break
            turret_time = us_ticker_read();
            yaw.setSpeed(0);
            pitch.setSpeed(0);
            yaw.pidSpeed.resetErrorIntegral();
            pitch.pidSpeed.resetErrorIntegral();
            return;
        }

        int dir = 0;
        if      (deltaYaw >  0.0f) dir = 1;
        else if (deltaYaw <  0.0f) dir = -1;
        
        yaw.pidPosition.feedForward = -chassis_rpm * 2 * PI / 60;

        // We calculate des velo instead of immediately calculating power to add to the feedforward of the speed PID
        float des_yaw_velo = yaw.pidPosition.calculatePeriodic(
            deltaYaw, 
            static_cast<float>(us_ticker_read() - turret_time) / 1000
        );
        yaw.pidSpeed.feedForward = yaw_static_friction * dir + yaw_kinetic_friction * des_yaw_velo;
        float des_yaw_power = yaw.pidSpeed.calculate(
            des_yaw_velo, 
            turret_state.yaw_velo_rad_s, 
            static_cast<float>(us_ticker_read() - turret_time) / 1000);
        printf("yp %.2f | %.2f\n", des_yaw_power, deltaYaw);
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
        float des_pitch_velo = pitch.pidPosition.calculate(
            forward_ * des_pitch, 
            forward_ * turret_state.pitch_angle_degs, 
            static_cast<float>(us_ticker_read() - turret_time) / 1000
        );
        pitch.setSpeed(des_pitch_velo);
        float des_pitch_power = pitch.pidSpeed.calculate(des_pitch_velo, turret_state.pitch_velo_rad_s, static_cast<float>(us_ticker_read() - turret_time) / 1000);
        printf("pp %.2f | %.2f\n", des_pitch_power, des_pitch-turret_state.pitch_angle_degs);
        // pitch.setPower(des_pitch_power);
    }

    turret_time = us_ticker_read();
}
