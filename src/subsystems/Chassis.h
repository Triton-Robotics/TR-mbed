#include "../../util/motor/CANMotor.hpp"

class Chassis {
    private:
        CANMotor LF, RF, LB, RB;
        CANMotor motors[4];

        double rpmToTicksPerSecond(double RPM);
        double ticksPerSecondToRPM(double ticksPerSecond);
        double rpmToInchesPerSecond(double RPM);
        double inchesPerSecondToRPM(double inchesPerSecond);

        void setMotorPower(int index, double power);
        void setMotorSpeedRPM(int index, double speed);
        void setMotorSpeedInchesPerSecond(int index, double speed);

    public:
        Chassis();
        void periodic();
        void tankDrive(double leftVelocityRPM, double rightVelocityRPM);
        void drive1(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
        void drive2(double angleRadians, double speedRPM, double rotationVelcotiyRPM);
        CANMotor getMotor(int index);

};