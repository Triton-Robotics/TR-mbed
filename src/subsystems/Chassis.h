#include "../../util/motor/CANMotor.hpp"

#ifndef Chassis_h
#define Chassis_h
enum BrakeMode {
    BRAKE,
    COAST
};

class Chassis {
    public:
        Chassis(short lfId, short rfId, short lbId, short rbId);

        void driveXYR(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
        void driveFieldRelative(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
        void driveOffsetAngle(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM, double angleOffset);
        void driveAngle(double angleRadians, double speedRPM, double rotationVelcotiyRPM);
        CANMotor getMotor(int index);

        BrakeMode getBrakeMode();
        void setBrakeMode(BrakeMode brakeMode);

    private:
        CANMotor LF, RF, LB, RB;
        BrakeMode brakeMode;

        double rpmToTicksPerSecond(double RPM);
        double ticksPerSecondToRPM(double ticksPerSecond);

        void setMotorPower(int index, double power);
        void setMotorSpeedRPM(int index, double speed);
        void setMotorSpeedTicksPerSecond(int index, double speed);
};
#endif