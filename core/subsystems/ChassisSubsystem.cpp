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

    LF.setSpeedPID(1.5, 0, 0);
    RF.setSpeedPID(1.5, 0, 0);
    LB.setSpeedPID(1.5, 0, 0);
    RB.setSpeedPID(1.5, 0, 0);
    brakeMode = BRAKE;

    // isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}

void ChassisSubsystem::setBrakeMode(BrakeMode brakeMode)
{

    return;
}

DJIMotor ChassisSubsystem::getMotor(int index)
{
    switch (index)
    {
    case 1:
        return LF;
    case 2:
        return RF;
    case 3:
        return LB;
    case 4:
        return RB;
    }
}

// BrakeMode ChassisSubsystem::getBrakeMode()
//{
//   return
//}

void ChassisSubsystem::readImu()
{
    imu.get_angular_position_quat(&imuAngles);
}

double ChassisSubsystem::degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

int ChassisSubsystem::getHeadingDegrees()
{
    return (int)imuAngles.yaw;
}

OmniKinematics ChassisSubsystem::setOmniKinematics(double radius)
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

WheelSpeeds ChassisSubsystem::ChassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
{
    return {(1 / sqrt(2)) * (chassisSpeeds.x + chassisSpeeds.y + chassisSpeeds.rotation * ((m_OmniKinematics.r1x) - (m_OmniKinematics.r1y))),
            (1 / sqrt(2)) * (chassisSpeeds.x - chassisSpeeds.y - chassisSpeeds.rotation * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
            (1 / sqrt(2)) * (-chassisSpeeds.x + chassisSpeeds.y + chassisSpeeds.rotation * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
            (1 / sqrt(2)) * (-chassisSpeeds.x - chassisSpeeds.y - chassisSpeeds.rotation * ((m_OmniKinematics.r4x) - (m_OmniKinematics.r4y)))};
}

void ChassisSubsystem::setMotorPower(int index, double power)
{
    DJIMotor motor = getMotor(index);
    motor.setPower(power);
}

void ChassisSubsystem::setMotorSpeedRPM(int index, double speed)
{
    DJIMotor motor = getMotor(index);
    motor.setSpeed(speed);
}
