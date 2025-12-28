#include "OmniWheelSubsystem.h"

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
    radius = cfg.radius;
    power_limit = cfg.power_limit;

    dt = (us_ticker_read() - prev_time) / 1000;
    prev_time = us_ticker_read();

    setYawReference(cfg.turret, cfg.initial_angle, cfg.yawAlign);

    calculateAccelLimit();
    setOmniKinematics();
}

void OmniWheelSubsystem::setChassisState(ChassisState state)
{
    desired_state = state;
}

OmniWheelSubsystem::ChassisState OmniWheelSubsystem::getChassisState()
{
    return curr_state;
}

void OmniWheelSubsystem::setYawReference(TurretSubsystem *_turret, float initial_angle, float _yawAlign)
{
    turret = _turret;
    yawAlign = _yawAlign;
    yawPhase = initial_angle;
}

bool OmniWheelSubsystem::setOdomReference()
{
    yawOdom = turret->getState().yaw_angle;
    imuOdom = imuAngles.yaw;
    return true;
}

void OmniWheelSubsystem::periodic()
{
    prev_state = curr_state;
    prev_wheelspeed = curr_wheelspeed;

    getWheelSpeed();
    getOmniState();

    setDesiredWheelSpeed();

    sendPower();
}


void OmniWheelSubsystem::setDesiredWheelSpeed()
{
    double yawCurrent = turret->getState().yaw_angle;
    ChassisSpeed desiredChassisSpeeds;

    if (curr_state.mode == YAW_ORIENTED)
    {
        // printf("%f\n", double(yaw->getData(ANGLE)));
        desiredChassisSpeeds = rotateChassisSpeed(desired_state.vel, yawCurrent);
    }
    else if (curr_state.mode == BEYBLADE)
    {
        // TODO: add maximum possible beyblade to this chassisspeeds

        // printf("%f\n", double(yaw->getData(ANGLE)));
        desiredChassisSpeeds = rotateChassisSpeed(desired_state.vel, yawCurrent);
    }
    else if (curr_state.mode == ROBOT_ORIENTED)
    {
        desiredChassisSpeeds = desired_state.vel; // ChassisSpeeds in m/s
    }
    else if (curr_state.mode == ODOM_ORIENTED)
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
    }
    else if (curr_state.mode == YAW_ALIGNED)
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
        float omegaCmd = (gain_align * yawError * deg2rad + gain_yaw * turret->getState().yaw_velo);

        if (abs(omegaCmd) < 0.1) omegaCmd = 0;

        ChassisSpeed xAlignSpeeds = {desired_state.vel.vX, desired_state.vel.vY, omegaCmd};

        desiredChassisSpeeds = rotateChassisSpeed(xAlignSpeeds, yawCurrent);
    }
    else 
    {
        desiredChassisSpeeds = {0.0, 0.0, 0.0};
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

void OmniWheelSubsystem::sendPower()
{
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
        prev_wheelspeed.fl,
        prev_wheelspeed.fr,
        prev_wheelspeed.bl,
        prev_wheelspeed.br
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