//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_CHASSIS_H
#define TR_EMBEDDED_CHASSIS_H

#include <motor/DJIMotor.h>
#include <communications/newCANHandler.h>


class Chassis {
public:
    Chassis(short lfId, short rfId, short lbId, short rbId);

    enum BrakeMode {
        BRAKE,
        COAST
    };

    void driveXYR(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
    void driveFieldRelative(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
    void driveOffsetAngle(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM, double angleOffset);
    void driveAngle(double angleRadians, double speedRPM, double rotationVelcotiyRPM);
    DJIMotor getMotor(int index);

    BrakeMode getBrakeMode();
    void setBrakeMode(BrakeMode brakeMode);

    int8_t isInverted[4];

private:
    DJIMotor LF, RF, LB, RB;
    BrakeMode brakeMode;

    double rpmToTicksPerSecond(double RPM);
    double ticksPerSecondToRPM(double ticksPerSecond);

    void setMotorPower(int index, double power);
    void setMotorSpeedRPM(int index, double speed);
    void setMotorSpeedTicksPerSecond(int index, double speed);
};

#endif //TR_EMBEDDED_CHASSIS_H
