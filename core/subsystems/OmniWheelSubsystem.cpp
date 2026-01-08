#include "OmniWheelSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "util/motor/DJIMotor.h"

OmniWheelSubsystem::OmniWheelSubsystem()
{
    curr_state = {
        OFF
    };
}

OmniWheelSubsystem::OmniWheelSubsystem(config cfg)
{
    PID::config pos_config;
    DJIMotor::config flcfg = 
    {
        cfg.flid,
        cfg.canBus,
        M3508,
        "Front Left",
        cfg.fl_vel_config,
        pos_config,
    };
    fl = DJIMotor(flcfg);
    
    DJIMotor::config frcfg = 
    {
        cfg.frid,
        cfg.canBus,
        M3508,
        "Front Right",
        cfg.fr_vel_config,
        pos_config,
    };
    fr = DJIMotor(frcfg);
    
    DJIMotor::config blcfg = 
    {
        cfg.blid,
        cfg.canBus,
        M3508,
        "Back Left",
        cfg.bl_vel_config,
        pos_config,
    };
    bl = DJIMotor(blcfg);
    
    DJIMotor::config brcfg = 
    {
        cfg.brid,
        cfg.canBus,
        M3508,
        "Back Right",
        cfg.br_vel_config,
        pos_config,
    };
    br = DJIMotor(brcfg);

    imu = cfg.imu;
    imuAngles = imu->getImuAngles();
    
    radius = cfg.radius;
    power_limit = cfg.power_limit;

    // TODO: figure out how to set a good maxVel and maxAccel
    // maxVel = cfg.max_vel;
    maxVel = -1.24 + 0.0513 * cfg.power_limit + -0.000216 * (cfg.power_limit * cfg.power_limit);
    maxAccel = cfg.max_accel;
    FF_Ks = cfg.feed_forward;
    maxBeyblade = cfg.max_beyblade;

    dt = (us_ticker_read() - prev_time) / 1000;
    prev_time = us_ticker_read();

    setYawReference(cfg.yaw, cfg.initial_angle, cfg.yawAlign);

    calculateAccelLimit();
    setOmniKinematics();
}

void OmniWheelSubsystem::setChassisState(ChassisState state)
{
    // Convert state velocity from a range of [-1,1] to real values
    desired_state.vel.vX = state.vel.vX * maxVel;
    desired_state.vel.vY = state.vel.vY * maxVel;
    desired_state.vel.vOmega = state.vel.vOmega * maxBeyblade;

    desired_state.mode = state.mode;
}

OmniWheelSubsystem::ChassisState OmniWheelSubsystem::getChassisState()
{
    return curr_state;
}

void OmniWheelSubsystem::setYawReference(TurretSubsystem *_yaw, float initial_angle, float _yawAlign)
{
    yaw = _yaw;
    yawAlign = _yawAlign;
    yawPhase = initial_angle;
}

bool OmniWheelSubsystem::setOdomReference()
{
    yawOdom = yaw->getTicks();
    imuOdom = imuAngles.yaw;
    return true;
}

void OmniWheelSubsystem::periodic()
{
    // Update curr_state and curr_wheelspeed
    imuAngles = imu->getImuAngles();
    getOmniState();

    sendPower();
}

void OmniWheelSubsystem::getOmniState()
{
    // Updating wheelspeeds
    curr_wheelspeed.fl = fl.getData(VELOCITY);
    curr_wheelspeed.fr = fr.getData(VELOCITY);
    curr_wheelspeed.bl = bl.getData(VELOCITY);
    curr_wheelspeed.br = br.getData(VELOCITY);

    calculateChassisSpeed();

    curr_state.mode = desired_state.mode;
}

void OmniWheelSubsystem::setDesiredWheelSpeed()
{
    double yawCurrent = yaw->getTicks();
    ChassisSpeed desiredChassisSpeeds;

    switch (curr_state.mode)
    {
    case YAW_ORIENTED:
    {
        desiredChassisSpeeds = rotateChassisSpeed(desired_state.vel, yawCurrent);
        break;
    }
    case BEYBLADE:
    {
        // TODO: change maxBeyblade dynamically based on keypress (add another state?)
        float jx = desired_state.vel.vX / maxVel;
        float jy = desired_state.vel.vY / maxVel;
        float linear_hypo = sqrtf(jx * jx + jy * jy);

        if(linear_hypo > 1.0){
            linear_hypo = 1.0;
        }

        desired_state.vel.vOmega = maxBeyblade * (1.0 - linear_hypo);

        desiredChassisSpeeds = rotateChassisSpeed(desired_state.vel, yawCurrent);
        break;
    }
    case ROBOT_ORIENTED:
    {
        desiredChassisSpeeds = desired_state.vel; // ChassisSpeeds in m/s
        break;
    }
    case ODOM_ORIENTED:
    {
        double yawDelta = yawOdom - yawCurrent;
        double imuDelta = imuOdom - imuAngles.yaw;
        double delta = imuDelta - yawDelta;
        double del = yawOdom + delta;
        while (del > 360.0)
            del -= 360;
        while (del < 0)
            del += 360;
        desiredChassisSpeeds = rotateChassisSpeed(desired_state.vel, yawOdom + delta);
        break;
    }
    case YAW_ALIGNED:
    {
        // Compute yaw error(how much the yaw needs to recorrect)
        float yawError = (yawCurrent - yawAlign);
        while (yawError > 180) yawError -= 360;
        while (yawError < -180) yawError += 360;
        
        if (abs(yawError) < 5) yawError = 0;

        if ((yawError >= 45 && yawError < 135)) {
            yawError -= 90;
        }
        if ((yawError >= 135)) {
            yawError -= 180;
        }
        if (yawError < -135) {
            yawError += 180;
        }
        if ((yawError >= -135 && yawError < -45)) {
            yawError += 90;
        }

        //tune these two for optimal performance
        float gain_align = 2;
        float gain_yaw = 3;
        float deg2rad = PI/180; // convert to rad and just run at 2x that rad/s
        float omegaCmd = (gain_align * yawError * deg2rad + gain_yaw * yaw->getTicks());

        if (abs(omegaCmd) < 0.1) omegaCmd = 0;

        ChassisSpeed xAlignSpeeds = {desired_state.vel.vX, desired_state.vel.vY, omegaCmd};

        desiredChassisSpeeds = rotateChassisSpeed(xAlignSpeeds, yawCurrent);
        break;
    }
    case OFF: 
    {
        desiredChassisSpeeds = {0.0, 0.0, 0.0};
        break;
    }
    }

    calculateWheelSpeed(desiredChassisSpeeds); // in m/s
}

void OmniWheelSubsystem::calculateWheelSpeed(ChassisSpeed chassisSpeeds)
{
    float SQRT_2 = sqrt(2);
    desired_wheelspeed = {(chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * (radius * SQRT_2)),
            (-chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * (radius * SQRT_2)),
            (chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * (radius * SQRT_2)),
            (-chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * (radius * SQRT_2))};
    
}

// TODO: Verify calculations PLEASE
void OmniWheelSubsystem::calculateChassisSpeed()
{
    float dist = radius / sqrt(2);
    float vX = (-curr_wheelspeed.fl - curr_wheelspeed.fr + curr_wheelspeed.bl + curr_wheelspeed.br) / 4;
    float vY = (-curr_wheelspeed.fl + curr_wheelspeed.fr - curr_wheelspeed.bl + curr_wheelspeed.br) / 4;
    float vOmega = (-curr_wheelspeed.fl - curr_wheelspeed.fr - curr_wheelspeed.bl - curr_wheelspeed.br) / (4 * (2 * dist));

    // Convert from RPM to m/s and rad/s
    vX = (vX / M3508_GEAR_RATIO) * (2 * PI / 60) * (WHEEL_DIAMETER_METERS / 2);
    vY = (vY / M3508_GEAR_RATIO) * (2 * PI / 60) * (WHEEL_DIAMETER_METERS / 2);
    vOmega = (vOmega / M3508_GEAR_RATIO) * (2 * PI / 60) * (WHEEL_DIAMETER_METERS / 2);
    
    curr_state.vel = {vX, vY, vOmega};
}

void OmniWheelSubsystem::sendPower()
{
    // Set the correct desired_wheelspeed and desired power
    setDesiredWheelSpeed();

    std::array<DJIMotor*, 4> motors = { &fl, &fr, &bl, &br };
    std::array<double, 4> desired = 
    {
        desired_wheelspeed.fl,
        desired_wheelspeed.fr,
        desired_wheelspeed.bl,
        desired_wheelspeed.br
    };
    std::array<double, 4> previous = 
    {
        curr_wheelspeed.fl,
        curr_wheelspeed.fr,
        curr_wheelspeed.bl,
        curr_wheelspeed.br
    };

    for (size_t i = 0; i < 4; ++i) 
    {
        // Find power output needed to run these motors
        motor_power[i] = motors[i]->calculateSpeedPID
        (
            desired[i],
            motors[i]->getData(VELOCITY),
            dt
        );

        if (desired[i] != 0.0) 
        {
            double sign = desired[i] / std::abs(desired[i]);
            motors[i]->pidSpeed.feedForward = sign * FF_Ks;
        } 
        else 
        {
            motors[i]->pidSpeed.feedForward = 0.0;
        }

        // Limit the accel
        float diff = desired[i] - previous[i];
        if ((desired[i] > 0 && previous[i] < 0) || (desired[i] < 0 && previous[i] > 0))
        { // if robot trying to sudden change direction
            desired[i] = 0;
        }
        if (diff > maxAccel)
        { // if the difference is greater than the max acceleration
            if (motor_power[i] != 0)
            {
                motor_power[i] = previous[i] + maxAccel;
            }
        }
        else if (diff < -maxAccel)
        {
            if (motor_power[i] != 0)
            {
                motor_power[i] = previous[i] - maxAccel;
            }
        }

        // Recalculate the power needed
        motor_power[i] = motors[i]->calculateSpeedPID
        (
            desired[i],
            motors[i]->getData(VELOCITY),
            dt
        );

        if (desired[i] != 0.0) 
        {
            double sign = desired[i] / std::abs(desired[i]);
            motors[i]->pidSpeed.feedForward = sign * FF_Ks;
        } 
        else 
        {
            motors[i]->pidSpeed.feedForward = 0.0;
        }
    }

    // I lowk cba to do this, make someone else do it plz :3
    float scale = Bisection
    (
        motor_power[0], 
        motor_power[1], 
        motor_power[2], 
        motor_power[3], 
        fl.getData(VELOCITY), 
        fr.getData(VELOCITY), 
        bl.getData(VELOCITY), 
        br.getData(VELOCITY), 
        power_limit
    );
    motor_power[0] *= scale;
    motor_power[1] *= scale;
    motor_power[2] *= scale;
    motor_power[3] *= scale;

    fl.setPower(motor_power[0]);
    fr.setPower(motor_power[1]);
    bl.setPower(motor_power[2]);
    br.setPower(motor_power[3]);
}

float OmniWheelSubsystem::Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm, float chassisPowerLimit)
{
    float scale = 0.5;      // initial scale
    float precision = 0.25; // initial precision
    float powerInit = p_theory(LeftFrontPower, RightFrontPower, LeftBackPower, RightBackPower, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);

    if (powerInit > chassisPowerLimit)
    {

        float powerScaled = p_theory(LeftFrontPower * scale, RightFrontPower * scale, LeftBackPower * scale, RightBackPower * scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);

        for (int i = 0; i < 6; i++)
        {

            if (powerScaled > chassisPowerLimit)
            {
                scale = scale - precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower * scale, RightFrontPower * scale, LeftBackPower * scale, RightBackPower * scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Down: %f\n", powerScaled);
            }

            else
            { // power is low enough
                scale = scale + precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower * scale, RightFrontPower * scale, LeftBackPower * scale, RightBackPower * scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Up: %f\n", powerScaled);
            }
        }
        return scale;
    }

    else
    {
        return 1;
    }
}

float OmniWheelSubsystem::p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm)
{
    float krpm2 = 0.000000000616869908524917;
    float kpwr2 = 2.8873053310419543e-26;
    float kboth = 0.00000000679867734389254;
    float a = 0.019247609510979;

    float p1 = (kboth * LeftFrontPower * LeftFrontRpm) + (krpm2 * LeftFrontRpm * LeftFrontRpm) + (kpwr2 * LeftFrontPower * LeftFrontPower) + a;
    float p2 = (kboth * RightFrontPower * RightFrontRpm) + (krpm2 * RightFrontRpm * RightFrontRpm) + (kpwr2 * RightFrontPower * RightFrontPower) + a;
    float p3 = (kboth * LeftBackPower * LeftBackRpm) + (krpm2 * LeftBackRpm * LeftBackRpm) + (kpwr2 * LeftBackPower * LeftBackPower) + a;
    float p4 = (kboth * RightBackPower * RightBackRpm) + (krpm2 * RightBackRpm * RightBackRpm) + (kpwr2 * RightBackPower * RightBackPower) + a;

    float p_tot = p1 + p2 + p3 + p4;

    float A = 224.9;
    float B = 215.8;
    float C = 0.7955;

    float p_tot_c = (A * p_tot * p_tot) + (B * p_tot) + C;

    return p_tot_c;
}