#include "TurretSubsystem.h"

TurretSubsystem::TurretSubsystem()
{
    configured = false;
}

TurretSubsystem::TurretSubsystem(config configuration)
{
    yaw = DJIMotor(configuration.yaw_id, configuration.yawCanBus, M3508);
    pitch = DJIMotor(configuration.pitch_id, configuration.pitchCanBus, M3508_FLYWHEEL);

    yaw.setSpeedPID(configuration.yaw_vel_PID.kp,
                    configuration.yaw_vel_PID.ki,
                    configuration.yaw_vel_PID.kd);

    yaw.setPositionPID(configuration.yaw_pos_PID.kp,
                    configuration.yaw_pos_PID.ki,
                    configuration.yaw_pos_PID.kd);
    
    pitch.setSpeedPID(configuration.pitch_vel_PID.kp,
                    configuration.pitch_vel_PID.ki,
                    configuration.pitch_vel_PID.kd);

    pitch.setPositionPID(configuration.pitch_vel_PID.kp,
                    configuration.pitch_vel_PID.ki,
                    configuration.pitch_vel_PID.kd);
    
    pitch_offset_ticks = configuration.pitch_offset_ticks;

    turretState = SLEEP;

    configured = true;
}

void TurretSubsystem::init(config configuration)
{
    if (configured == false)
    {
        yaw = DJIMotor(configuration.yaw_id, configuration.yawCanBus, M3508);
        pitch = DJIMotor(configuration.pitch_id, configuration.pitchCanBus, M3508_FLYWHEEL);

        yaw.setSpeedPID(configuration.yaw_vel_PID.kp,
                        configuration.yaw_vel_PID.ki,
                        configuration.yaw_vel_PID.kd);

        yaw.setPositionPID(configuration.yaw_pos_PID.kp,
                        configuration.yaw_pos_PID.ki,
                        configuration.yaw_pos_PID.kd);
        
        pitch.setSpeedPID(configuration.pitch_vel_PID.kp,
                        configuration.pitch_vel_PID.ki,
                        configuration.pitch_vel_PID.kd);

        pitch.setPositionPID(configuration.pitch_vel_PID.kp,
                        configuration.pitch_vel_PID.ki,
                        configuration.pitch_vel_PID.kd);
        
        pitch_offset_ticks = configuration.pitch_offset_ticks;

        turretState = SLEEP;

        configured = true;
    }
}

TurretSubsystem::TurretInfo TurretSubsystem::getState()
{
    return turret_state;
}

void TurretSubsystem::setState (TurretState des_state)
{
    turretState = des_state;
}

double TurretSubsystem::get_pitch_angle_rads_zero_offsetted()
{
    return 0.0;
}

double TurretSubsystem::get_pitch_vel_rads_per_sec()
{
    return 0.0;
}

double TurretSubsystem::get_yaw_vel_rads_per_sec()
{
    return 0.0;
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
        yaw.pidSpeed.feedForward = chassis_rpm;
        yaw.setPosition(des_yaw);
        pitch.setPosition(des_pitch);
    }
    return;
}

void TurretSubsystem::periodic()
{
    // TODO do we recalculate our values here?
    execute_turret();
    
    // Update turret_state here!
    turret_state.yaw_angle = yaw.getData(ANGLE);
    turret_state.pitch_angle = get_pitch_angle_rads_zero_offsetted();
    turret_state.yaw_velo = get_yaw_vel_rads_per_sec();
    turret_state.pitch_velo = get_pitch_vel_rads_per_sec();

    turret_time = us_ticker_read();
}
