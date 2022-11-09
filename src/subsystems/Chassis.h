#include "../../util/motor/CANMotor.hpp"

#ifndef Chassis_h
#define Chassis_h
class Chassis {
    public:
        Chassis();

        void driveXYR(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
        void driveFieldRelative(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
        void driveAngle(double angleRadians, double speedRPM, double rotationVelcotiyRPM);
        CANMotor getMotor(int index);

    private:
        CANMotor LF, RF, LB, RB;

        double rpmToTicksPerSecond(double RPM);
        double ticksPerSecondToRPM(double ticksPerSecond);

        void setMotorPower(int index, double power);
        void setMotorSpeedRPM(int index, double speed);
        void setMotorSpeedTicksPerSecond(int index, double speed);
};
#endif