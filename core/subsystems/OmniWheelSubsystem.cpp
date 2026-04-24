#include "OmniWheelSubsystem.h"
#include "util/motor/DJIMotor.h"
#include <cmath>
#include <stdexcept>
#include <us_ticker_defines.h>

/**
 * @param radius radius in meters
 */
OmniWheelSubsystem::OmniWheelSubsystem(const Config &config, MA4 *encoder_)
    : power_limit(config.power_limit),
      LF(DJIMotor::config({
        short(config.left_front_can_id),
        CAN_BUS_TYPE,
        MOTOR_TYPE,
        "left_front",
        config.lf_pid
      })),
      RF(DJIMotor::config({
        short(config.right_front_can_id),
        CAN_BUS_TYPE,
        MOTOR_TYPE,
        "right_front",
        config.rf_pid
      })),
      LB(DJIMotor::config({
        short(config.left_back_can_id),
        CAN_BUS_TYPE,
        MOTOR_TYPE,
        "left_back",
        config.lb_pid
      })),
      RB(DJIMotor::config({
        short(config.right_back_can_id),
        CAN_BUS_TYPE,
        MOTOR_TYPE,
        "right_back",
        config.rb_pid
      })),
      encoder(encoder_),
      yawPhase{config.yaw_initial_offset_ticks},
      chassis_radius(config.radius),
      FF_Ks(config.speed_pid_ff_ks)
{
    LF.outputCap = 16000;
    RF.outputCap = 16000;
    LB.outputCap = 16000;
    RB.outputCap = 16000;

    setOmniKinematics(config.radius, config.chassis_type);
    m_OmniKinematicsLimits.max_Vel = MAX_VEL; // m/s
    m_OmniKinematicsLimits.max_vOmega = 8; // rad/s
}

WheelSpeeds OmniWheelSubsystem::getWheelSpeeds() const
{
    return m_wheelSpeeds;
}

float OmniWheelSubsystem::limitAcceleration(float desiredRPM, float previousRPM, unsigned long deltaTime, float theta)
{
        float diff = desiredRPM - previousRPM;

    // Calculate theoretical max acceleration
    float trigDenom = 2 * max(abs(sin(theta + M_PI/4)), abs(sin(theta + M_PI/4)));
    float maxLinearAccel = (STATIC_FRICTION_CONSTANT * GRAVITY) / (ACCEL_DENOM_CONSTANT * trigDenom);

    // Maximum change in velocity over this time period, then change that to RPM
    float maxChange = maxLinearAccel * (deltaTime / 1000000.0);
    float maxChangeRPM = maxChange * ((1 / (WHEEL_DIAMETER_METERS / 2) / (2 * PI / 60) * M3508_GEAR_RATIO));
    
    if ((desiredRPM > 0 && previousRPM < 0) || (desiredRPM < 0 && previousRPM > 0)) { // if robot trying to sudden change direction
        return 0;
    }
    
    if (diff > maxChangeRPM) {
        return previousRPM + maxChangeRPM;
    } 
    else if (diff < -maxChangeRPM) { // Also check deceleration
        return previousRPM - maxChangeRPM;
    } 
    else { // Under acceleration limit
        return desiredRPM;
    }
}

float OmniWheelSubsystem::curr_fit(int x) 
{
    if ((static_cast<float>(x)) / 5596 > 0.4375) {
        if ((us_ticker_read() - last_torque_time) / 1000 > 200) {
            last_torque_time = us_ticker_read();
            return 1.22;
        }
        // printf("%.3f ", static_cast<float>(x) / 5596);
    }
    return (abs(static_cast<float>(x)) /5596) * (14/4.9);
}

float OmniWheelSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds)
{
    desiredWheelSpeeds = wheelSpeeds; // WheelSpeeds in RPM
    int powers[4] = {0,0,0,0};
    uint32_t time = us_ticker_read();
    uint32_t deltaTime = time - lastPIDTime;

    float diffLF = wheelSpeeds.LF - previousRPM[0];
    float diffRF = wheelSpeeds.RF - previousRPM[1];
    float diffLB = wheelSpeeds.LB - previousRPM[2];
    float diffRB = wheelSpeeds.RB - previousRPM[3];


    float accelX = diffLF + diffRF - diffLB - diffRB;
    float accelY = diffLF - diffRF + diffLB - diffRB;
    float theta = atan2(accelY, accelX) + M_PI/2;

    
    float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], deltaTime, theta);
    float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], deltaTime, theta);
    float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], deltaTime, theta);
    float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], deltaTime, theta);
    
    powers[0] = M3508_GEAR_RATIO * LF.calculateSpeedPID(LFrpm, previousRPM[0], (time - lastPIDTime));
    powers[1] = M3508_GEAR_RATIO * RF.calculateSpeedPID(RFrpm, previousRPM[1], (time - lastPIDTime));
    powers[2] = M3508_GEAR_RATIO * LB.calculateSpeedPID(LBrpm, previousRPM[2], (time - lastPIDTime));
    powers[3] = M3508_GEAR_RATIO * RB.calculateSpeedPID(RBrpm, previousRPM[3], (time - lastPIDTime));

    printf("%.2f %.2f %.2f %.2f\n", LFrpm, RFrpm, LBrpm, RBrpm);
    
    lastPIDTime = time;
    previousRPM[0] = getMotorSpeed(LEFT_FRONT, RPM) / M3508_GEAR_RATIO;
    previousRPM[1] = getMotorSpeed(RIGHT_FRONT, RPM) / M3508_GEAR_RATIO;
    previousRPM[2] = getMotorSpeed(LEFT_BACK, RPM) / M3508_GEAR_RATIO;
    previousRPM[3] = getMotorSpeed(RIGHT_BACK, RPM) / M3508_GEAR_RATIO;

    float current_est = 24 * (curr_fit(LF.getData(TORQUE)) + 
                              curr_fit(RF.getData(TORQUE)) + 
                              curr_fit(LB.getData(TORQUE)) + 
                              curr_fit(RB.getData(TORQUE)));

    
    double scale = power_limit / (current_est + 10); // underestimate power :)
    
    if (scale >=1) scale = 1;

    LF.setPower(powers[0]*scale);
    RF.setPower(powers[1]*scale);
    LB.setPower(powers[2]*scale);
    RB.setPower(powers[3]*scale);

    return scale;
}

WheelSpeeds OmniWheelSubsystem::normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const
{
    double speeds[4] = {wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB};
    double max_speed = m_OmniKinematicsLimits.max_Vel;

    for (double speed : speeds)
        if (speed > max_speed)
            max_speed = speed;

    if (max_speed > m_OmniKinematicsLimits.max_Vel)
        for (double &speed : speeds)
            speed = speed / max_speed * m_OmniKinematicsLimits.max_Vel;

    return {speeds[0], speeds[1], speeds[2], speeds[3]};
}


ChassisSpeeds OmniWheelSubsystem::getChassisSpeeds() const
{
    return m_chassisSpeeds;
}

float OmniWheelSubsystem::setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode)
{
    double yawCurrent = 0;
    if (mode == YAW_ORIENTED)
    {
        yawCurrent = 360 - encoder->encoderMovingAverage();
        if (yawCurrent < 0.0) {
            yawCurrent += 360.0;
        }
        else if (yawCurrent > 360.0) {
            yawCurrent -= 360.0;
        }
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == ROBOT_ORIENTED)
    {
        desiredChassisSpeeds = desiredChassisSpeeds_; // ChassisSpeeds in m/s
    }
    else if (mode == ODOM_ORIENTED) 
    {
        yawCurrent = 360 - encoder->encoderMovingAverage();
        if (yawCurrent < 0.0) {
            yawCurrent += 360.0;
        }
        else if (yawCurrent > 360.0) {
            yawCurrent -= 360.0;
        }

        double yawDelta = yawOdom - yawCurrent;
        double imuDelta = imuOdom - imuAngles.yaw;
        double delta = imuDelta - yawDelta;
        double del = yawOdom + delta;
        while (del > 360.0) del -= 360;
        while (del < 0) del += 360;
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawOdom + delta);
    }
    else if (mode == YAW_ALIGN)
    {
        desiredChassisSpeeds = desiredChassisSpeeds_; // TEMP TODO FIX
    }
    WheelSpeeds wheelSpeeds = chassisSpeedsToWheelSpeeds(desiredChassisSpeeds); // in m/s (for now)
    wheelSpeeds = normalizeWheelSpeeds(wheelSpeeds);
    wheelSpeeds *= (1 / (WHEEL_DIAMETER_METERS / 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
    float scale = setWheelSpeeds(wheelSpeeds);
    return scale;
}

ChassisSpeeds OmniWheelSubsystem::rotateChassisSpeed(ChassisSpeeds speeds, double yawCurrent)
{
    // rotate angle counter clockwise
    double theta = (yawCurrent - yawPhase) / 180 * PI;
    return {speeds.vX * cos(theta) - speeds.vY * sin(theta),
            speeds.vX * sin(theta) + speeds.vY * cos(theta),
            speeds.vOmega};
}

DJIMotor &OmniWheelSubsystem::getMotor(MotorLocation location)
{
    switch (location)
    {
    case LEFT_FRONT:
        return LF;
    case RIGHT_FRONT:
        return RF;
    case LEFT_BACK:
        return LB;
    case RIGHT_BACK:
        return RB;
    }
    
    assert(false && "Invalid MotorLocation");
    __builtin_unreachable();
}

void OmniWheelSubsystem::setMotorSpeedPID(MotorLocation location, float kP, float kI, float kD)
{
    getMotor(location).setSpeedPID(kP, kI, kD);
}

void OmniWheelSubsystem::setSpeedFeedforward(MotorLocation location, double FF)
{
    // getMotor(location).pidSpeed.feedForward = FF * INT15_T_MAX;
    if(location == LEFT_FRONT){
        LF.pidSpeed.feedForward = FF * INT15_T_MAX;
    }else if(location == LEFT_BACK){
        LB.pidSpeed.feedForward = FF * INT15_T_MAX;
    }else if(location == RIGHT_FRONT){
        RF.pidSpeed.feedForward = FF * INT15_T_MAX;
    }else if(location == RIGHT_BACK){
        RB.pidSpeed.feedForward = FF * INT15_T_MAX;
    }
}


double OmniWheelSubsystem::getMotorSpeed(MotorLocation location, SPEED_UNIT unit = RPM)
{
    double speed = getMotor(location).getData(VELOCITY);
    switch (unit)
    {
    case RPM:
        return (speed * M3508_GEAR_RATIO) / (2 * PI / 60);
    case METER_PER_SECOND:
        return speed * (WHEEL_DIAMETER_METERS / 2);
    case RAD_PER_SECOND:
        return speed;
    }

    assert(false && "Invalid motor speed unit");
    __builtin_unreachable();
}

void OmniWheelSubsystem::periodic(IMU::EulerAngles *imuCurr)
{   
    imuAngles.yaw = imuCurr->yaw;
    imuAngles.pitch = imuCurr->pitch;
    imuAngles.roll = imuCurr->roll;
    m_wheelSpeeds = {getMotorSpeed(LEFT_FRONT, METER_PER_SECOND), getMotorSpeed(RIGHT_FRONT, METER_PER_SECOND),
                     getMotorSpeed(LEFT_BACK, METER_PER_SECOND), getMotorSpeed(RIGHT_BACK, METER_PER_SECOND)};

    m_chassisSpeeds = wheelSpeedsToChassisSpeeds(m_wheelSpeeds);
}

double OmniWheelSubsystem::degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

double OmniWheelSubsystem::radiansToDegrees(double radians)
{
    return radians / PI * 180.0;
}

void OmniWheelSubsystem::setOmniKinematicsLimits(double max_Vel, double max_vOmega)
{
    m_OmniKinematicsLimits.max_Vel = max_Vel;
    m_OmniKinematicsLimits.max_vOmega = max_vOmega;
}

void OmniWheelSubsystem::setOmniKinematics(double radius, HOLONOMIC_MODE mode)
{
    if (mode == OMNI) {
        float SQRT_2 = sqrt(2);
        m_OmniKinematics.r1x = radius/SQRT_2;
        m_OmniKinematics.r1y = radius/SQRT_2;

        m_OmniKinematics.r2x = radius/SQRT_2;
        m_OmniKinematics.r2y = radius/SQRT_2;

        m_OmniKinematics.r3x = radius/SQRT_2;
        m_OmniKinematics.r3y = radius/SQRT_2;

        m_OmniKinematics.r4x = radius/SQRT_2;
        m_OmniKinematics.r4y = radius/SQRT_2;
    } 

    else if (mode == MECANUM) {
        m_OmniKinematics.r1x = WHEEL_TO_CHASSIS_CENTER_LY;
        m_OmniKinematics.r1y = WHEEL_TO_CHASSIS_CENTER_LX;

        m_OmniKinematics.r2x = WHEEL_TO_CHASSIS_CENTER_LY;
        m_OmniKinematics.r2y = WHEEL_TO_CHASSIS_CENTER_LX;

        m_OmniKinematics.r3x = WHEEL_TO_CHASSIS_CENTER_LY;
        m_OmniKinematics.r3y = WHEEL_TO_CHASSIS_CENTER_LX;

        m_OmniKinematics.r4x = WHEEL_TO_CHASSIS_CENTER_LY;
        m_OmniKinematics.r4y = WHEEL_TO_CHASSIS_CENTER_LX;
    }
}

//inputs chassis speeds in m/s, outputs wheel speeds in m/s
WheelSpeeds OmniWheelSubsystem::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
{
    return {(+chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r1x) + (m_OmniKinematics.r1y))),
            (-chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
            (+chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
            (-chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r4x) + (m_OmniKinematics.r4y)))};
}

ChassisSpeeds OmniWheelSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
{
    float dist = chassis_radius/sqrt(2);
    float vX = (wheelSpeeds.LF + wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vY = (wheelSpeeds.LF - wheelSpeeds.RF + wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vOmega = (-wheelSpeeds.LF - wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / (4*(2 * dist));
    return {vX, vY, vOmega};
}


bool OmniWheelSubsystem::setOdomReference() {
    // yawOdom = -(1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0;
    imuOdom = imuAngles.yaw;
    return true;
}

void OmniWheelSubsystem::updateYawPhaseFromEncoder() {
    float encoder_reading = 360 - encoder->encoderMovingAverage();
        if (encoder_reading < 0.0) {
            encoder_reading += 360.0;
        }
        else if (encoder_reading > 360.0) {
            encoder_reading -= 360.0;
        }
    if (encoder_reading >= 0) {
        yawPhase = encoder_reading;
    }
}

double OmniWheelSubsystem::radiansToTicks(double radians)
{
    return radians / (2 * PI) * TICKS_PER_ROTATION;
}

double OmniWheelSubsystem::ticksToRadians(double ticks)
{
    return ticks / TICKS_PER_ROTATION * (2 * PI);
}

double OmniWheelSubsystem::rpmToRadPerSecond(double RPM)
{
    return RPM * (2 * PI) / SECONDS_PER_MINUTE;
}

double OmniWheelSubsystem::radPerSecondToRPM(double radPerSecond)
{
    return radPerSecond / (2 * PI) * SECONDS_PER_MINUTE;
}