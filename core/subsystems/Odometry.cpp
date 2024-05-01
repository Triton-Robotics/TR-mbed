#include "Odometry.h"
#include <cmath>
#include <stdexcept>

/**
 * @param radius radius in meters
 */
Odometry::Odometry(ChassisSubsystem &chassis)
        : chassis(chassis)
{
    // LF.outputCap = 16000; // DJIMotor class has a max outputCap: 16384
    // RF.outputCap = 16000;
    // LB.outputCap = 16000;
    // RB.outputCap = 16000;

    // this->lfId = lfId;
    // this->rfId = rfId;
    // this->lbId = lbId;
    // this->rbId = rbId;

    // setOmniKinematics(radius);
    // m_OmniKinematicsLimits.max_Vel = 2.92; // m/s
    // m_OmniKinematicsLimits.max_vOmega = 2; // rad/s

    // FF_Ks = 0;

    // LF.setSpeedPID(2, 0, 0);
    // RF.setSpeedPID(2, 0, 0);
    // LB.setSpeedPID(2, 0, 0);
    // RB.setSpeedPID(2, 0, 0);

    // brakeMode = COAST;

    // isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}

void Odometry::odom_shout()
{
    
}

// WheelSpeeds ChassisSubsystem::getWheelSpeeds() const
// {
//     return m_wheelSpeeds;
// }

// void ChassisSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds)
// {
//     desiredWheelSpeeds = wheelSpeeds; // WheelSpeeds in RPM
//     setMotorSpeedRPM(LEFT_FRONT, wheelSpeeds.LF);
//     setMotorSpeedRPM(RIGHT_FRONT, wheelSpeeds.RF);
//     setMotorSpeedRPM(LEFT_BACK, wheelSpeeds.LB);
//     setMotorSpeedRPM(RIGHT_BACK, wheelSpeeds.RB);
// }

// WheelSpeeds ChassisSubsystem::normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const
// {
//     double speeds[4] = {wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB};
//     double max_speed = m_OmniKinematicsLimits.max_Vel;

//     for (double speed : speeds)
//         if (speed > max_speed)
//             max_speed = speed;

//     if (max_speed > m_OmniKinematicsLimits.max_Vel)
//         for (double &speed : speeds)
//             speed = speed / max_speed * m_OmniKinematicsLimits.max_Vel;

//     return {speeds[0], speeds[1], speeds[2], speeds[3]};
// }

// void ChassisSubsystem::setWheelPower(WheelSpeeds wheelPower)
// {
//     setMotorPower(LEFT_FRONT, wheelPower.LF);
//     setMotorPower(RIGHT_FRONT, wheelPower.RF);
//     setMotorPower(LEFT_BACK, wheelPower.LB);
//     setMotorPower(RIGHT_BACK, wheelPower.RB);
// }

// ChassisSpeeds ChassisSubsystem::getChassisSpeeds() const
// {
//     return m_chassisSpeeds;
// }

// /**
//  * There's no setChassisPower because it doesn't make sense.
//  * Power (is not PWM voltage) saturates your motor speeds, and it's not related to motor speed. 
//  */

// ChassisSpeeds ChassisSubsystem::rotateChassisSpeed(ChassisSpeeds speeds, double yawCurrent)
// {
//     // rotate angle counter clockwise
//     double theta = (yawCurrent - yawPhase) / 180 * PI;
//     return {speeds.vX * cos(theta) - speeds.vY * sin(theta),
//             speeds.vX * sin(theta) + speeds.vY * cos(theta),
//             speeds.vOmega};
// }

// DJIMotor &ChassisSubsystem::getMotor(MotorLocation location)
// {
//     switch (location)
//     {
//     case LEFT_FRONT:
//         return LF;
//     case RIGHT_FRONT:
//         return RF;
//     case LEFT_BACK:
//         return LB;
//     case RIGHT_BACK:
//         return RB;
//     }
// }

// void ChassisSubsystem::setMotorSpeedPID(MotorLocation location, float kP, float kI, float kD)
// {
//     getMotor(location).setSpeedPID(kP, kI, kD);
// }

// void ChassisSubsystem::setSpeedIntegralCap(MotorLocation location, double cap)
// {
//     getMotor(location).setSpeedIntegralCap(cap);
// }

// void ChassisSubsystem::setSpeedFeedforward(MotorLocation location, double FF)
// {
//     getMotor(location).pidSpeed.feedForward = FF * INT15_T_MAX;
// }

// void ChassisSubsystem::setSpeedFF_Ks(double Ks)
// {
//     FF_Ks = Ks;
// }

// ChassisSubsystem::BrakeMode ChassisSubsystem::getBrakeMode()
// {
//     return brakeMode;
// }

// void ChassisSubsystem::setBrakeMode(BrakeMode brakeMode)
// {
//     this->brakeMode = brakeMode;
// }

// void ChassisSubsystem::initializeImu()
// {
//     imu.set_mounting_position(MT_P1);
// }

// double ChassisSubsystem::getMotorSpeed(MotorLocation location, SPEED_UNIT unit = RPM)
// {
//     double speed = getMotor(location).getData(VELOCITY);
//     switch (unit)
//     {
//     case RPM:
//         return speed;
//     case METER_PER_SECOND:
//         return speed / M3508_GEAR_RATIO * (2 * PI / 60) * WHEEL_DIAMETER_METERS / 2;
//     }
// }

// void ChassisSubsystem::readImu()
// {
//     imu.get_angular_position_quat(&imuAngles);
// }

// void ChassisSubsystem::periodic()
// {   
//     readImu();
//     m_wheelSpeeds = {getMotorSpeed(LEFT_FRONT, METER_PER_SECOND), getMotorSpeed(RIGHT_FRONT, METER_PER_SECOND),
//                      getMotorSpeed(LEFT_BACK, METER_PER_SECOND), getMotorSpeed(RIGHT_BACK, METER_PER_SECOND)};

//     m_chassisSpeeds = wheelSpeedsToChassisSpeeds(m_wheelSpeeds);
// }

// double ChassisSubsystem::degreesToRadians(double degrees)
// {
//     return degrees * PI / 180.0;
// }

// double ChassisSubsystem::radiansToDegrees(double radians)
// {
//     return radians / PI * 180.0;
// }

// int ChassisSubsystem::getHeadingDegrees() const
// {
//     return (int)imuAngles.yaw;
// }

// void ChassisSubsystem::setOmniKinematicsLimits(double max_Vel, double max_vOmega)
// {
//     m_OmniKinematicsLimits.max_Vel = max_Vel;
//     m_OmniKinematicsLimits.max_vOmega = max_vOmega;
// }

// void ChassisSubsystem::setOmniKinematics(double radius)
// {
//     m_OmniKinematics.r1x = -sqrt(radius);
//     m_OmniKinematics.r1y = sqrt(radius);

//     m_OmniKinematics.r2x = sqrt(radius);
//     m_OmniKinematics.r2y = sqrt(radius);

//     m_OmniKinematics.r3x = -sqrt(radius);
//     m_OmniKinematics.r3y = -sqrt(radius);

//     m_OmniKinematics.r4x = sqrt(radius);
//     m_OmniKinematics.r4y = -sqrt(radius);
// }

// WheelSpeeds ChassisSubsystem::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
// {
//     return {(1 / sqrt(2)) * (chassisSpeeds.vX + chassisSpeeds.vY + chassisSpeeds.vOmega * ((m_OmniKinematics.r1x) - (m_OmniKinematics.r1y))),
//             (1 / sqrt(2)) * (chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
//             (1 / sqrt(2)) * (-chassisSpeeds.vX + chassisSpeeds.vY + chassisSpeeds.vOmega * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
//             (1 / sqrt(2)) * (-chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r4x) - (m_OmniKinematics.r4y)))};
// }

// ChassisSpeeds ChassisSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
// {
//     Eigen::MatrixXd Inv_K(4, 3);
//     float coef = 1 / sqrt(2);
//     Inv_K << coef, coef, coef * ((m_OmniKinematics.r1x) - (m_OmniKinematics.r1y)),
//         coef, -coef, -coef * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y)),
//         -coef, coef, coef * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y)),
//         -coef, -coef, -coef * ((m_OmniKinematics.r4x) - (m_OmniKinematics.r4y));
//     Eigen::MatrixXd FWD_K(3, 4);
//     FWD_K = Inv_K.completeOrthogonalDecomposition().pseudoInverse();
//     Eigen::MatrixXd WS(4, 1);
//     WS << wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB;
//     Eigen::MatrixXd CS(3, 1);
//     CS = FWD_K * WS;
//     return {CS(0, 0), CS(1, 0), CS(2, 0)};
// }

// char *ChassisSubsystem::MatrixtoString(Eigen::MatrixXd mat)
// {
//     std::stringstream ss;
//     ss << mat;
//     ss << '\0';
//     ss << '\n';
//     const char *a = ss.str().c_str();
//     return strdup(a);
// }