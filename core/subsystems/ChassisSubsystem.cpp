#include "ChassisSubsystem.h"
#include <cmath>
#include <stdexcept>

/**
 * @param radius radius in meters
 */
ChassisSubsystem::ChassisSubsystem(const Config &config)
    : power_limit(60),
      LF(config.left_front_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      RF(config.right_front_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      LB(config.left_back_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      RB(config.right_back_can_id, CAN_BUS_TYPE, MOTOR_TYPE),
      encoder(config.encoder),
      yawPhase{config.yaw_initial_offset_ticks}, // change Yaw to CCW +, and ranges from 0 to 360
      imu(config.imu),
      chassis_radius(config.radius),
      FF_Ks(config.speed_pid_ff_ks)
{
    LF.outputCap = 16000; // DJIMotor class has a max outputCap: 16384
    RF.outputCap = 16000;
    LB.outputCap = 16000;
    RB.outputCap = 16000;

    setOmniKinematics(config.radius);
    m_OmniKinematicsLimits.max_Vel = MAX_VEL; // m/s
    m_OmniKinematicsLimits.max_vOmega = 8; // rad/s

    PEAK_POWER_ALL = 10000;
    PEAK_POWER_SINGLE = 8000;

//    LF.setSpeedPID(2, 0, 0);
//    RF.setSpeedPID(2, 0, 0);
//    LB.setSpeedPID(2, 0, 0);
//    RB.setSpeedPID(2, 0, 0);
    // LF.setSpeedPID(3, 0, 0);
    // RF.setSpeedPID(3, 0, 0);
    // LB.setSpeedPID(3, 0, 0);
    // RB.setSpeedPID(3 , 0, 0);

    // Keeping original PID gains (3, 0, 0)
    pid_LF.setPID(3, 0, 0);
    pid_RF.setPID(3, 0, 0);
    pid_LB.setPID(3, 0, 0);
    pid_RB.setPID(3, 0, 0);

    brakeMode = COAST;

    // isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}

WheelSpeeds ChassisSubsystem::getWheelSpeeds() const
{
    return m_wheelSpeeds;
}

float ChassisSubsystem::limitAcceleration(float desiredRPM, float previousRPM, int power)
{
    float maxAccel = 100;
    float diff = desiredRPM - previousRPM;

    if ((desiredRPM > 0 && previousRPM < 0) || (desiredRPM < 0 && previousRPM > 0)) { // if robot trying to sudden change direction
        return 0;
    }

    if (diff > maxAccel){   // if the difference is greater than the max acceleration
        if(power == 0) {
            return desiredRPM; // let robot do its thing b/c it wont take power
        }
        return previousRPM + maxAccel;
    }
    else if (diff < -maxAccel) {
        if(power == 0) {
            return desiredRPM; // let robot do its thing b/c it wont take power
        }
        return previousRPM - maxAccel;
    }
    else {
        return desiredRPM; // under acceleration cap
    }
}

double ChassisSubsystem::p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm){
    double krpm2 = 0.000000000616869908524917;
    double kpwr2 = 2.8873053310419543e-26;
    double kboth = 0.00000000679867734389254;
    double a = 0.019247609510979;


    double p1 =  (kboth * LeftFrontPower * LeftFrontRpm) +  (krpm2 * LeftFrontRpm * LeftFrontRpm) +  (kpwr2 * LeftFrontPower * LeftFrontPower) + a;
    double p2 =  (kboth * RightFrontPower * RightFrontRpm) +  (krpm2 * RightFrontRpm * RightFrontRpm) +  (kpwr2 * RightFrontPower * RightFrontPower) + a;
    double p3 =  (kboth * LeftBackPower * LeftBackRpm) +  (krpm2 * LeftBackRpm * LeftBackRpm) +  (kpwr2 * LeftBackPower * LeftBackPower) + a;
    double p4 =  (kboth * RightBackPower * RightBackRpm) +  (krpm2 * RightBackRpm * RightBackRpm) +  (kpwr2 * RightBackPower * RightBackPower) + a;
    
    double p_tot = p1 + p2 + p3 + p4; 

    double A = 224.9;
    double B = 215.8;
    double C = 0.7955;

    double p_tot_c = (A * p_tot * p_tot) + (B * p_tot) + C;

    return p_tot_c;
}

double ChassisSubsystem::Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm, float chassisPowerLimit) {
    double scale = 0.5; // initial scale
    double precision = 0.25; // initial precision
    double powerInit = p_theory(LeftFrontPower, RightFrontPower, LeftBackPower, RightBackPower, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);


    if (powerInit > chassisPowerLimit) {

        double powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
        
        for (int i = 0; i < 6; i++) {
            if (powerScaled > chassisPowerLimit) {
                scale = scale - precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Down: %f\n", powerScaled);
            }
            else {
                scale = scale + precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Up: %f\n", powerScaled);
            }
        }
        return scale;
    }
    else {
        return 1;
    }
}

float ChassisSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds)
{
    desiredWheelSpeeds = wheelSpeeds; // WheelSpeeds in RPM
    int powers[4] = {0,0,0,0};
    uint32_t time = us_ticker_read();

    powers[0] = motorPIDtoPower(LEFT_FRONT,wheelSpeeds.LF, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT,wheelSpeeds.RF, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK,wheelSpeeds.LB, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK,wheelSpeeds.RB, (time - lastPIDTime));

    float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], powers[0]);
    float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], powers[1]);
    float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], powers[2]);
    float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], powers[3]);

    previousRPM[0] = LFrpm;
    previousRPM[1] = RFrpm;
    previousRPM[2] = LBrpm;
    previousRPM[3] = RBrpm;

    powers[0] = motorPIDtoPower(LEFT_FRONT,LFrpm, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT,RFrpm, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK,LBrpm, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK,RBrpm, (time - lastPIDTime));
    lastPIDTime = time;

    int p1 = abs(powers[0]);
    int p2 = abs(powers[1]);
    int p3 = abs(powers[2]);
    int p4 = abs(powers[3]);

    int r1 = abs(LF.getData(VELOCITY));
    int r2 = abs(RF.getData(VELOCITY));
    int r3 = abs(LB.getData(VELOCITY));
    int r4 = abs(RB.getData(VELOCITY));

    double scale = Bisection(p1, p2, p3, p4, r1, r2, r3, r4, power_limit);

    // Debug print: RPMs, bisection scale, and predicted power before motors are set
    printf("RPM LF:%.1f RF:%.1f LB:%.1f RB:%.1f | Scale:%.4f | Power:%.2f\n",
        LFrpm, RFrpm, LBrpm, RBrpm, scale,
        p_theory(p1, p2, p3, p4, r1, r2, r3, r4));

    // printf("Before Set:%.3f\n", p_theory(p1*scale, p2*scale, p3*scale, p4*scale, r1, r2, r3, r4));

    

    LF.setPower(powers[0]*scale);
    RF.setPower(powers[1]*scale);
    LB.setPower(powers[2]*scale);
    RB.setPower(powers[3]*scale);

    p1 = abs(LF.getData(POWEROUT));
    p2 = abs(RF.getData(POWEROUT));
    p3 = abs(LB.getData(POWEROUT));
    p4 = abs(RB.getData(POWEROUT));

    // printf("After Set:%.3f\n", p_theory(p1, p2, p3, p4, r1, r2, r3, r4));

    return scale;
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


float ChassisSubsystem::computeMaxOmega(float vX, float vY) const
{
    static constexpr float P_IDLE       = 74.97f; // idle power floor (W)
    static constexpr float K_SPIN       = 2.38f;  // W per rad/s
    static constexpr float K_TRANS_AXIAL = 5.14f; // W per m/s, axial motion
    static constexpr float K_TRANS_DIAG  = 2.19f; // W per m/s, diagonal motion
    static constexpr float P_BUDGET     = 90.0f;  // power budget (W)

    float vXY = sqrtf(vX*vX + vY*vY);

    bool  is_moving   = (vXY > 0.01f);
    float axial_ratio = is_moving ? fmaxf(fabsf(vX), fabsf(vY)) / vXY : 0.0f;

    float K_TRANS = K_TRANS_DIAG + (K_TRANS_AXIAL - K_TRANS_DIAG) * axial_ratio;

    float power_available = P_BUDGET - P_IDLE - K_TRANS * vXY;

    float vOmega_max = power_available / K_SPIN;

    return fmaxf(0.0f, vOmega_max);
}

float ChassisSubsystem::setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode)
{

    float vOmega_max = computeMaxOmega(desiredChassisSpeeds_.vX, desiredChassisSpeeds_.vY);

    static float vOmega_smoothed = 0.0f;
    static constexpr float OMEGA_RATE_LIMIT = 0.15f;
    float vOmega_target = fminf(fabsf(desiredChassisSpeeds_.vOmega), vOmega_max)
                          * (desiredChassisSpeeds_.vOmega >= 0 ? 1.0f : -1.0f);
    float vOmega_delta  = vOmega_target - vOmega_smoothed;
    vOmega_delta = fmaxf(-OMEGA_RATE_LIMIT, fminf(OMEGA_RATE_LIMIT, vOmega_delta));
    vOmega_smoothed += vOmega_delta;

    ChassisSpeeds adjusted = desiredChassisSpeeds_;
    adjusted.vOmega = vOmega_smoothed;

    if (mode == REVERSE_YAW_ORIENTED)
    {
        // printf("%f\n", double(yaw->getData(ANGLE)));
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0) {
            yawCurrent += 360.0;
        }
        else if (yawCurrent > 360.0) {
            yawCurrent -= 360.0;
        }
        // if (yawCurrent == -1.0) {
        //     yawCurrent = (1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0; // change Yaw to CCW +, and ranges from 0 to 360
        // }
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == YAW_ORIENTED)
    {
        // printf("%f\n", double(yaw->getData(ANGLE)));
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0) {
            yawCurrent += 360.0;
        }
        else if (yawCurrent > 360.0) {
            yawCurrent -= 360.0;
        }
        // if (yawCurrent == -1.0) {
        //     yawCurrent = (1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0; // change Yaw to CCW +, and ranges from 0 to 360
        // }
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == ROBOT_ORIENTED)
    {
        desiredChassisSpeeds = adjusted;
    }
    else if (mode == ODOM_ORIENTED)
    {
        yawCurrent = encoder->encoderMovingAverage();
        if (yawCurrent < 0.0) {
            yawCurrent += 360.0;
        }
        else if (yawCurrent > 360.0) {
            yawCurrent -= 360.0;
        }
        // if (yawCurrent == -1.0) {
        //     yawCurrent = (1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0; // change Yaw to CCW +, and ranges from 0 to 360
        // }
        double yawDelta = yawOdom - yawCurrent;
        double imuDelta = imuOdom - imuAngles.yaw;
        double delta = imuDelta - yawDelta;
        double del = yawOdom + delta;
        while (del > 360.0) del -= 360;
        while (del < 0) del += 360;
        desiredChassisSpeeds = rotateChassisSpeed(adjusted, yawOdom + delta);
    }

    WheelSpeeds wheelSpeeds = chassisSpeedsToWheelSpeeds(desiredChassisSpeeds);
    wheelSpeeds = normalizeWheelSpeeds(wheelSpeeds);
    wheelSpeeds *= (1 / (WHEEL_DIAMETER_METERS / 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
    float scale = setWheelSpeeds(wheelSpeeds);

    static int logCounter = 0;
    if (++logCounter >= 25) {
        logCounter = 0;
        float vX  = desiredChassisSpeeds_.vX;
        float vY  = desiredChassisSpeeds_.vY;
        float vXY = sqrtf(vX*vX + vY*vY);
        printf("vX:%.3f vY:%.3f |vXY|:%.3f vOmegaMax:%.3f vW:%.3f | LF:%.2f RF:%.2f LB:%.2f RB:%.2f Total:%.2f W\n",
            vX, vY, vXY,
            vOmega_max,
            desiredChassisSpeeds_.vOmega,
            m_wheelPowers.LF, m_wheelPowers.RF,
            m_wheelPowers.LB, m_wheelPowers.RB,
            m_wheelPowers.total);
    }

    return scale;
}

/**
 * There's no setChassisPower because it doesn't make sense.
 * Power (is not PWM voltage) saturates your motor speeds, and it's not related to motor speed.
 */

ChassisSpeeds ChassisSubsystem::rotateChassisSpeed(ChassisSpeeds speeds, double yawCurrent)
{
    // rotate angle counter clockwise
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

    assert(false && "Invalid MotorLocation");
    __builtin_unreachable();
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
    // getMotor(location).pidSpeed.feedForward = FF * INT15_T_MAX;
    if(location == LEFT_FRONT){
        pid_LF.feedForward = FF * INT15_T_MAX;
    }else if(location == LEFT_BACK){
        pid_LB.feedForward = FF * INT15_T_MAX;
    }else if(location == RIGHT_FRONT){
        pid_RF.feedForward = FF * INT15_T_MAX;
    }else if(location == RIGHT_BACK){
        pid_RB.feedForward = FF * INT15_T_MAX;
    }
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
    return;
    // imu.set_mounting_position(MT_P1);
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
    case RAD_PER_SECOND:
        // TODO this should be handled properly
        return 0;
    }

    assert(false && "Invalid motor speed unit");
    __builtin_unreachable();
}

void ChassisSubsystem::readImu()
{
    imu.getImuAngles();
}

void ChassisSubsystem::periodic(IMU::EulerAngles *imuCurr)
{
    imuAngles.yaw   = imuCurr->yaw;
    imuAngles.pitch = imuCurr->pitch;
    imuAngles.roll  = imuCurr->roll;
    m_wheelSpeeds = {getMotorSpeed(LEFT_FRONT, METER_PER_SECOND), getMotorSpeed(RIGHT_FRONT, METER_PER_SECOND),
                     getMotorSpeed(LEFT_BACK,  METER_PER_SECOND), getMotorSpeed(RIGHT_BACK,  METER_PER_SECOND)};

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
    m_OmniKinematicsLimits.max_Vel    = max_Vel;
    m_OmniKinematicsLimits.max_vOmega = max_vOmega;
}

void ChassisSubsystem::setOmniKinematics(double radius, HOLONOMIC_MODE mode)
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


WheelSpeeds ChassisSubsystem::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
{
    return {(chassisSpeeds.vY + chassisSpeeds.vX - chassisSpeeds.vOmega * ((m_OmniKinematics.r1x) + (m_OmniKinematics.r1y))),
            (chassisSpeeds.vY - chassisSpeeds.vX - chassisSpeeds.vOmega * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
            (-chassisSpeeds.vY + chassisSpeeds.vX - chassisSpeeds.vOmega * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
            (-chassisSpeeds.vY - chassisSpeeds.vX - chassisSpeeds.vOmega * ((m_OmniKinematics.r4x) + (m_OmniKinematics.r4y)))};
}

ChassisSpeeds ChassisSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
{
    float dist = chassis_radius/sqrt(2);
    float vX     = (wheelSpeeds.LF + wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vY     = (wheelSpeeds.LF - wheelSpeeds.RF + wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vOmega = (-wheelSpeeds.LF - wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / (4*(2 * dist));
    return {vX, vY, vOmega};
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
    PID* pids[4] = {&pid_LF, &pid_RF, &pid_LB, &pid_RB};
    
    power = pids[location]->calculate(speed, getMotorSpeed(location, RPM), dt);
    // printf("[%d]",power);

    if(speed == 0) {
        setSpeedFeedforward(location, 0);
        return power;
    }
    double sgn_speed = speed / abs(speed);
    setSpeedFeedforward(location, FF_Ks * sgn_speed);
    return power;
}

bool ChassisSubsystem::setOdomReference() {
    // yawOdom = -(1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0;
    imuOdom = imuAngles.yaw;
    return true;
}

// double ChassisSubsystem::getEncoderYawPosition() {
//     if (encoder == nullptr) {
//         return -1.0f;  // Encoder not available
//     }

//     static float filtered_yaw = 0.0f;
//     float filter_alpha = 0.2f;  // 0.0-1.0: lower = more smoothing, higher = more responsive

//     float duty_raw = 360 - encoder->dutycycle();
//     float duty_min = 0.02943f;   // 2.943%
//     float duty_max = 0.97058f;   // 97.058%

//     //low pass filter 
//     double yaw_position = (double)(abs(((duty_raw - duty_min) / (duty_max - duty_min)) * 360.0));
//     filtered_yaw = filtered_yaw * (1.0f - filter_alpha) + yaw_position *  filter_alpha;
//     // printf("%.2f\n",yaw_position);
//     return filtered_yaw;
// }

// double ChassisSubsystem::encoderMovingAverage() {
//     const int windowSize = 20;
//     static double readings[windowSize] = {0};
//     static int index = 0;
//     static bool filled = false;

//     double newReading = getEncoderYawPosition();
//     if (newReading < 0) {
//         return -1.0f; // Encoder not available
//     }

//     readings[index] = newReading;
//     index = (index + 1) % windowSize;
//     if (index == 0) {
//         filled = true;
//     }

//     double sum = 0.0;
//     double result = 0.0;

//     if (filled) {
//         for (int i = 0; i < windowSize; i++) {
//             sum += readings[i];
//         }
//         result = sum / windowSize;
//     } else {
//         for (int i = 0; i < index; i++) {
//             sum += readings[i];
//         }
//         result = sum / index;
//     }
//     // printf("%.2f\n",result);
//     return result;

// }

void ChassisSubsystem::updateYawPhaseFromEncoder() {
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