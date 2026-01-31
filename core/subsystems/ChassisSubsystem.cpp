#include "ChassisSubsystem.h"
#include <cmath>
#include <stdexcept>

/**
 * @param radius radius in meters
 */

ChassisSubsystem::ChassisSubsystem(short lfId, short rfId, short lbId, short rbId, BNO055 &imu, double radius)
    : LF(lfId, CAN_BUS_TYPE, MOTOR_TYPE),
      RF(rfId, CAN_BUS_TYPE, MOTOR_TYPE),
      LB(lbId, CAN_BUS_TYPE, MOTOR_TYPE),
      RB(rbId, CAN_BUS_TYPE, MOTOR_TYPE),
      imu(imu),
      power_limit(50.0F),
      chassis_radius(radius)
{
    LF.outputCap = 16000;
    RF.outputCap = 16000;
    LB.outputCap = 16000;
    RB.outputCap = 16000;

    this->lfId = lfId;
    this->rfId = rfId;
    this->lbId = lbId;
    this->rbId = rbId;

    setOmniKinematics(radius);
    m_OmniKinematicsLimits.max_Vel = MAX_VEL;
    m_OmniKinematicsLimits.max_vOmega = 8;

    PEAK_POWER_ALL = 10000;
    PEAK_POWER_SINGLE = 8000;

    FF_Ks = 0;

    pid_LF.setPID(3, 0, 0);
    pid_RF.setPID(3, 0, 0);
    pid_LB.setPID(3, 0, 0);
    pid_RB.setPID(3, 0, 0);

    brakeMode = COAST;

}

WheelSpeeds ChassisSubsystem::getWheelSpeeds() const
{
    return m_wheelSpeeds;
}


// NEW: PI measured-power limiter


void ChassisSubsystem::setPowerLimiterGains(float kP, float kI, float alpha)
{
    m_kP = kP;
    m_kI = kI;
    m_alpha = clampf(alpha, 0.0f, 1.0f);
}

void ChassisSubsystem::updatePowerLimiter(float measuredPower_W, float dt_s)
{
    if (dt_s <= 0.0f) return;
    if (dt_s > 0.2f) dt_s = 0.2f;

    // 1) low-pass filter
    m_powerFiltered_W += m_alpha * (measuredPower_W - m_powerFiltered_W);

    // 2) error (positive = under limit, negative = over limit)
    float error = power_limit - m_powerFiltered_W;

    // 3) integrator with anti-windup
    m_powerIntegral += error * dt_s;
    m_powerIntegral = clampf(m_powerIntegral, -2000.0f, 2000.0f);

    // 4) PI update scale
    float delta = (m_kP * error) + (m_kI * m_powerIntegral);
    m_powerScale += delta;

    // 5) clamp
    m_powerScale = clampf(m_powerScale, 0.0f, 1.0f);
}


float ChassisSubsystem::limitAcceleration(float desiredRPM, float previousRPM, int power){
    float maxAccel = 100;
    float diff = desiredRPM - previousRPM;

    if ((desiredRPM > 0 && previousRPM < 0) || (desiredRPM < 0 && previousRPM > 0)){
        return 0;
    }

    if (diff > maxAccel){
        if (power == 0){
            return desiredRPM;
        }
        return previousRPM + maxAccel;
    }
    else if (diff < -maxAccel){
        if (power == 0){
            return desiredRPM;
        }
        return previousRPM - maxAccel;
    }
    else{
        return desiredRPM;
    }
}

// ===== old theory functions (kept, not used by new PI limiter) =====
float ChassisSubsystem::p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm){
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

// NOT NEEDED BY THE NEW ALGORITHM
float ChassisSubsystem::Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower,
                                  int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm,
                                  float chassisPowerLimit)
{
    float scale = 0.5;
    float precision = 0.25;
    float powerInit = p_theory(LeftFrontPower, RightFrontPower, LeftBackPower, RightBackPower, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);

    if (powerInit > chassisPowerLimit)
    {
        float powerScaled = p_theory(LeftFrontPower * scale, RightFrontPower * scale, LeftBackPower * scale, RightBackPower * scale,
                                     LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);

        for (int i = 0; i < 10; i++)
        {
            if (powerScaled > chassisPowerLimit)
                scale -= precision;
            else
                scale += precision;

            precision *= 0.5f;

            if (scale < 0.0f) scale = 0.0f;
            if (scale > 1.0f) scale = 1.0f;

            powerScaled = p_theory(LeftFrontPower * scale, RightFrontPower * scale, LeftBackPower * scale, RightBackPower * scale,
                                   LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
        }
        return scale;
    }
    else
    {
        return 1;
    }
}
// NOT NEEDED BY THE NEW ALGORITHM BECAUSE I AM TRYING TO USE PID

float ChassisSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds)
{
    desiredWheelSpeeds = wheelSpeeds; // RPM
    int powers[4] = {0, 0, 0, 0};
    uint32_t time = us_ticker_read();

    powers[0] = motorPIDtoPower(LEFT_FRONT, wheelSpeeds.LF, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT, wheelSpeeds.RF, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK, wheelSpeeds.LB, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK, wheelSpeeds.RB, (time - lastPIDTime));

    float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], powers[0]);
    float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], powers[1]);
    float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], powers[2]);
    float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], powers[3]);

    // float LFrpm = wheelSpeeds.LF;
    // float RFrpm = wheelSpeeds.RF;
    // float LBrpm = wheelSpeeds.LB;
    // float RBrpm = wheelSpeeds.RB;

    
    // float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], LF.getData(POWEROUT));
    // float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], RF.getData(POWEROUT));
    // float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], LB.getData(POWEROUT));
    // float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], RB.getData(POWEROUT));

    previousRPM[0] = LFrpm;
    previousRPM[1] = RFrpm;
    previousRPM[2] = LBrpm;
    previousRPM[3] = RBrpm;

    powers[0] = motorPIDtoPower(LEFT_FRONT, LFrpm, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT, RFrpm, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK, LBrpm, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK, RBrpm, (time - lastPIDTime));

    lastPIDTime = time;

    // ==========================
    // NEW: Apply PI power scale
    // ==========================

    int p1 = powers[0];
    int p2 = powers[1];
    int p3 = powers[2];
    int p4 = powers[3];

    int r1 = LF.getData(VELOCITY);
    int r2 = RF.getData(VELOCITY);
    int r3 = LB.getData(VELOCITY);
    int r4 = RB.getData(VELOCITY);
    
    //DONT REALLY NEED THIS BUT WILL LEAVE THIS HERE
    float scale = Bisection(p1, p2, p3, p4, r1, r2, r3, r4, power_limit);
    
    LF.setPower(powers[0] * m_powerScale);
    RF.setPower(powers[1] * m_powerScale);
    LB.setPower(powers[2] * m_powerScale);
    RB.setPower(powers[3] * m_powerScale);

    p1 = abs(LF.getData(POWEROUT));
    p2 = abs(RF.getData(POWEROUT));
    p3 = abs(LB.getData(POWEROUT));
    p4 = abs(RB.getData(POWEROUT));

    return m_powerScale;
}

WheelSpeeds ChassisSubsystem::normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const
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

void ChassisSubsystem::setWheelPower(WheelSpeeds wheelPower)
{
    setMotorPower(LEFT_FRONT, wheelPower.LF);
    setMotorPower(RIGHT_FRONT, wheelPower.RF);
    setMotorPower(LEFT_BACK, wheelPower.LB);
    setMotorPower(RIGHT_BACK, wheelPower.RB);
}

ChassisSpeeds ChassisSubsystem::getChassisSpeeds() const
{
    return m_chassisSpeeds;
}

float ChassisSubsystem::setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode)
{
    if (mode == REVERSE_YAW_ORIENTED)
    {
        double yawCurrent = (1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0;
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == YAW_ORIENTED)
    {
        double yawCurrent = -(1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0;
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else
    {
        desiredChassisSpeeds = desiredChassisSpeeds_;
    }

    WheelSpeeds wheelSpeeds = chassisSpeedsToWheelSpeeds(desiredChassisSpeeds);
    wheelSpeeds = normalizeWheelSpeeds(wheelSpeeds);
    wheelSpeeds *= (1 / (WHEEL_DIAMETER_METERS / 2) / (2 * PI / 60) * M3508_GEAR_RATIO);

    float scale = setWheelSpeeds(wheelSpeeds);
    return scale;
}

ChassisSpeeds ChassisSubsystem::rotateChassisSpeed(ChassisSpeeds speeds, double yawCurrent)
{
    double theta = (yawCurrent - yawPhase) / 180 * PI;
    return {speeds.vX * cos(theta) - speeds.vY * sin(theta),
            speeds.vX * sin(theta) + speeds.vY * cos(theta),
            speeds.vOmega};
}

DJIMotor &ChassisSubsystem::getMotor(MotorLocation location)
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
}

void ChassisSubsystem::setMotorSpeedPID(MotorLocation location, float kP, float kI, float kD)
{
    getMotor(location).setSpeedPID(kP, kI, kD);
}

void ChassisSubsystem::setSpeedIntegralCap(MotorLocation location, double cap)
{
    getMotor(location).setSpeedIntegralCap(cap);
}

void ChassisSubsystem::setSpeedFeedforward(MotorLocation location, double FF)
{
    if (location == LEFT_FRONT)
        pid_LF.feedForward = FF * INT15_T_MAX;
    else if (location == LEFT_BACK)
        pid_LB.feedForward = FF * INT15_T_MAX;
    else if (location == RIGHT_FRONT)
        pid_RF.feedForward = FF * INT15_T_MAX;
    else if (location == RIGHT_BACK)
        pid_RB.feedForward = FF * INT15_T_MAX;
}

void ChassisSubsystem::setSpeedFF_Ks(double Ks)
{
    FF_Ks = Ks;
}

ChassisSubsystem::BrakeMode ChassisSubsystem::getBrakeMode()
{
    return brakeMode;
}

void ChassisSubsystem::setBrakeMode(BrakeMode brakeMode)
{
    this->brakeMode = brakeMode;
}

void ChassisSubsystem::initializeImu()
{
    imu.set_mounting_position(MT_P1);
}

double ChassisSubsystem::getMotorSpeed(MotorLocation location, SPEED_UNIT unit = RPM)
{
    double speed = getMotor(location).getData(VELOCITY);
    switch (unit)
    {
    case RPM:
        return speed;
    case METER_PER_SECOND:
        return (speed / M3508_GEAR_RATIO) * (2 * PI / 60) * (WHEEL_DIAMETER_METERS / 2);
    default:
        return speed;
    }
}

void ChassisSubsystem::readImu()
{
    imu.get_angular_position_quat(&imuAngles);
}

void ChassisSubsystem::periodic()
{
    m_wheelSpeeds = {getMotorSpeed(LEFT_FRONT, METER_PER_SECOND),
                     getMotorSpeed(RIGHT_FRONT, METER_PER_SECOND),
                     getMotorSpeed(LEFT_BACK, METER_PER_SECOND),
                     getMotorSpeed(RIGHT_BACK, METER_PER_SECOND)};

    m_chassisSpeeds = wheelSpeedsToChassisSpeeds(m_wheelSpeeds);
}

double ChassisSubsystem::degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

double ChassisSubsystem::radiansToDegrees(double radians)
{
    return radians / PI * 180.0;
}

int ChassisSubsystem::getHeadingDegrees() const
{
    return (int)imuAngles.yaw;
}

void ChassisSubsystem::setOmniKinematicsLimits(double max_Vel, double max_vOmega)
{
    m_OmniKinematicsLimits.max_Vel = max_Vel;
    m_OmniKinematicsLimits.max_vOmega = max_vOmega;
}

void ChassisSubsystem::setOmniKinematics(double radius)
{
    float SQRT_2 = sqrt(2);
    m_OmniKinematics.r1x = radius / SQRT_2;
    m_OmniKinematics.r1y = radius / SQRT_2;

    m_OmniKinematics.r2x = radius / SQRT_2;
    m_OmniKinematics.r2y = radius / SQRT_2;

    m_OmniKinematics.r3x = radius / SQRT_2;
    m_OmniKinematics.r3y = radius / SQRT_2;

    m_OmniKinematics.r4x = radius / SQRT_2;
    m_OmniKinematics.r4y = radius / SQRT_2;
}

WheelSpeeds ChassisSubsystem::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
{
    return {(chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r1x) + (m_OmniKinematics.r1y))),
            (chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
            (-chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
            (-chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r4x) + (m_OmniKinematics.r4y)))};
}

ChassisSpeeds ChassisSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
{
    float dist = chassis_radius / sqrt(2);
    float vX = (wheelSpeeds.LF + wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vY = (wheelSpeeds.LF - wheelSpeeds.RF + wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vOmega = (-wheelSpeeds.LF - wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / (4 * (2 * dist));
    return {vX, vY, vOmega};
}

char *ChassisSubsystem::MatrixtoString(Eigen::MatrixXd mat)
{
    std::stringstream ss;
    ss << mat;
    ss << '\0';
    ss << '\n';
    const char *a = ss.str().c_str();
    return strdup(a);
}

void ChassisSubsystem::setMotorPower(MotorLocation location, double power)
{
    if (brakeMode == BRAKE && power == 0)
    {
        getMotor(location).setSpeed(0);
        setSpeedFeedforward(location, 0);
        return;
    }
    getMotor(location).setPower(power * INT15_T_MAX);
}

void ChassisSubsystem::setMotorSpeedRPM(MotorLocation location, double speed)
{
    if (brakeMode == COAST && speed == 0)
    {
        setMotorPower(location, 0);
        setSpeedFeedforward(location, 0);
        return;
    }
    getMotor(location).setSpeed(speed);
    double sgn_speed = speed / abs(speed);
    setSpeedFeedforward(location, FF_Ks * sgn_speed);
}

int ChassisSubsystem::motorPIDtoPower(MotorLocation location, double speed, uint32_t dt)
{
    if (brakeMode == COAST && speed == 0)
    {
        setSpeedFeedforward(location, 0);
        return 0;
    }
    int power = 0;
    
    PID *pid = nullptr;
    switch (location)
    {
    case LEFT_FRONT:
        pid = &pid_LF;
        break;
    case RIGHT_FRONT:
        pid = &pid_RF;
        break;
    case LEFT_BACK:
        pid = &pid_LB;
        break;
    case RIGHT_BACK:
        pid = &pid_RB;
        break;
    default:
        pid = &pid_LF;
        break;
    }

    int power = pid->calculate(speed, getMotor(location).getData(VELOCITY), dt);

    if (speed == 0)
    {
        setSpeedFeedforward(location, 0);
        return power;
    }

    double sgn_speed = speed / abs(speed);
    setSpeedFeedforward(location, FF_Ks * sgn_speed);
    return power;
}

void ChassisSubsystem::setYawReference(DJIMotor *motor, double initial_offset_ticks)
{
    yaw = motor;
    yawPhase = 360.0 * (1.0 - (initial_offset_ticks / TICKS_REVOLUTION));
}

double ChassisSubsystem::radiansToTicks(double radians)
{
    return radians / (2 * PI) * TICKS_PER_ROTATION;
}

double ChassisSubsystem::ticksToRadians(double ticks)
{
    return ticks / TICKS_PER_ROTATION * (2 * PI);
}

double ChassisSubsystem::rpmToRadPerSecond(double RPM)
{
    return RPM * (2 * PI) / SECONDS_PER_MINUTE;
}

double ChassisSubsystem::radPerSecondToRPM(double radPerSecond)
{
    return radPerSecond / (2 * PI) * SECONDS_PER_MINUTE;
}