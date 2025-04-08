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
      imu(imu)
// chassisKalman()
{
    LF.outputCap = 16000; // DJIMotor class has a max outputCap: 16384
    RF.outputCap = 16000;
    LB.outputCap = 16000;
    RB.outputCap = 16000;

    this->lfId = lfId;
    this->rfId = rfId;
    this->lbId = lbId;
    this->rbId = rbId;

    setOmniKinematics(radius);
    m_OmniKinematicsLimits.max_Vel = MAX_VEL; // m/s
    m_OmniKinematicsLimits.max_vOmega = 8; // rad/s

    PEAK_POWER_ALL = 10000;
    PEAK_POWER_SINGLE = 8000;

    FF_Ks = 0;

//    LF.setSpeedPID(2, 0, 0);
//    RF.setSpeedPID(2, 0, 0);
//    LB.setSpeedPID(2, 0, 0);
//    RB.setSpeedPID(2, 0, 0);
    // LF.setSpeedPID(3, 0, 0);
    // RF.setSpeedPID(3, 0, 0);
    // LB.setSpeedPID(3, 0, 0);
    // RB.setSpeedPID(3 , 0, 0);

    pid_LF.setPID(3.66, 0.0000, 0);
    pid_RF.setPID(3.66, 0.0000, 0);
    pid_LB.setPID(3.66, 0.0000, 0);
    pid_RB.setPID(3.66, 0.0000, 0);

    brakeMode = COAST;

    // isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}

WheelSpeeds ChassisSubsystem::getWheelSpeeds() const
{
    return m_wheelSpeeds;
}

void ChassisSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds)
{
    desiredWheelSpeeds = wheelSpeeds; // WheelSpeeds in RPM
    int powers[4] = {0,0,0,0};
    uint32_t time = us_ticker_read();
    powers[0] = motorPIDtoPower(LEFT_FRONT,wheelSpeeds.LF, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT,wheelSpeeds.RF, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK,wheelSpeeds.LB, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK,wheelSpeeds.RB, (time - lastPIDTime));
    lastPIDTime = time;

    int sum = abs(powers[0]) + abs(powers[1]) + abs(powers[2]) + abs(powers[3]);
    if(sum > PEAK_POWER_ALL){
        powers[0] = (powers[0] * PEAK_POWER_ALL)/sum;
        powers[1] = (powers[1] * PEAK_POWER_ALL)/sum;
        powers[2] = (powers[2] * PEAK_POWER_ALL)/sum;
        powers[3] = (powers[3] * PEAK_POWER_ALL)/sum;
    }
    LF.setPower(powers[0]);
    RF.setPower(powers[1]);
    LB.setPower(powers[2]);
    RB.setPower(powers[3]);
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

void ChassisSubsystem::setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode)
{
    if (mode == REVERSE_YAW_ORIENTED)
    {
        // printf("%f\n", double(yaw->getData(ANGLE)));
        double yawCurrent = (1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0; // change Yaw to CCW +, and ranges from 0 to 360
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == YAW_ORIENTED)
    {
        // printf("%f\n", double(yaw->getData(ANGLE)));
        double yawCurrent = -(1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0; // change Yaw to CCW +, and ranges from 0 to 360
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == ROBOT_ORIENTED)
    {
        desiredChassisSpeeds = desiredChassisSpeeds_; // ChassisSpeeds in m/s
    }
    WheelSpeeds wheelSpeeds = chassisSpeedsToWheelSpeeds(desiredChassisSpeeds); // in m/s (for now)
    wheelSpeeds = normalizeWheelSpeeds(wheelSpeeds);
    wheelSpeeds *= (1 / (WHEEL_DIAMETER_METERS / 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
    setWheelSpeeds(wheelSpeeds);
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
        return speed / M3508_GEAR_RATIO * (2 * PI / 60) * WHEEL_DIAMETER_METERS / 2;
    }
}

void ChassisSubsystem::readImu()
{
    imu.get_angular_position_quat(&imuAngles);
}

void ChassisSubsystem::periodic()
{   
    readImu();
    m_wheelSpeeds = {getMotorSpeed(LEFT_FRONT, METER_PER_SECOND), getMotorSpeed(RIGHT_FRONT, METER_PER_SECOND),
                     getMotorSpeed(LEFT_BACK, METER_PER_SECOND), getMotorSpeed(RIGHT_BACK, METER_PER_SECOND)};

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
    m_OmniKinematics.r1x = -sqrt(radius);
    m_OmniKinematics.r1y = sqrt(radius);

    m_OmniKinematics.r2x = sqrt(radius);
    m_OmniKinematics.r2y = sqrt(radius);

    m_OmniKinematics.r3x = -sqrt(radius);
    m_OmniKinematics.r3y = -sqrt(radius);

    m_OmniKinematics.r4x = sqrt(radius);
    m_OmniKinematics.r4y = -sqrt(radius);
}

WheelSpeeds ChassisSubsystem::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
{
    return {(1 / sqrt(2)) * (chassisSpeeds.vX + chassisSpeeds.vY + chassisSpeeds.vOmega * ((m_OmniKinematics.r1x) - (m_OmniKinematics.r1y))),
            (1 / sqrt(2)) * (chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
            (1 / sqrt(2)) * (-chassisSpeeds.vX + chassisSpeeds.vY + chassisSpeeds.vOmega * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
            (1 / sqrt(2)) * (-chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r4x) - (m_OmniKinematics.r4y)))};
}

ChassisSpeeds ChassisSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
{
    Eigen::MatrixXd Inv_K(4, 3);
    float coef = 1 / sqrt(2);
    Inv_K << coef, coef, coef * ((m_OmniKinematics.r1x) - (m_OmniKinematics.r1y)),
        coef, -coef, -coef * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y)),
        -coef, coef, coef * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y)),
        -coef, -coef, -coef * ((m_OmniKinematics.r4x) - (m_OmniKinematics.r4y));
    Eigen::MatrixXd FWD_K(3, 4);
    FWD_K = Inv_K.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd WS(4, 1);
    WS << wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB;
    Eigen::MatrixXd CS(3, 1);
    CS = FWD_K * WS;
    return {CS(0, 0), CS(1, 0), CS(2, 0)};
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
    if (brakeMode == BRAKE && power == 0) // Should be BRAKE
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
    double sgn_speed = speed / abs(speed); // if speed is 0, it won't execute this line
    setSpeedFeedforward(location, FF_Ks * sgn_speed);
}

int ChassisSubsystem::motorPIDtoPower(MotorLocation location, double speed, uint32_t dt)
{
    if (brakeMode == COAST && speed == 0)
    {
        setSpeedFeedforward(location, 0);
        return 0;
    }else{
        //double sgn_speed = speed / abs(speed); // if speed is 0, it won't execute this line
        //setSpeedFeedforward(location, FF_Ks * sgn_speed);
    }
    
    int power = 0;
    PID pids[4] = {pid_LF,pid_RF,pid_LB,pid_RB};
    
    power = pids[location].calculate(speed, getMotor(location).getData(VELOCITY), dt);
    // printf("[%d]",power);
    return power;
}

void ChassisSubsystem::setYawReference(DJIMotor *motor, double initial_offset_ticks)
{
    yaw = motor;
    yawPhase = 360.0 * (1.0 - (initial_offset_ticks / TICKS_REVOLUTION)); // change Yaw to CCW +, and ranges from 0 to 360
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