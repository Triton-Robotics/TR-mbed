#include "Chassis.h"

#define PI 3.14159265

#define CAN_BUS_TYPE NewCANHandler::CANBUS_1
#define MOTOR_TYPE M3508

double getAngleRadians() {
    return PI / 2.0;
}

Chassis::Chassis() : LF(4, CAN_BUS_TYPE, MOTOR_TYPE), RF(1, CAN_BUS_TYPE, MOTOR_TYPE), 
    LB(2, CAN_BUS_TYPE, MOTOR_TYPE), RB(3, CAN_BUS_TYPE, MOTOR_TYPE) {
    LF.outCap = 16000;   
    RF.outCap = 16000;   
    LB.outCap = 16000;   
    RB.outCap = 16000;   
    LF.setSpeedPID(0.8, 0.2, 0);
    RF.setSpeedPID(0.9, 0.25, 0);
    LB.setSpeedPID(0.9, 0.2, 0);
    RB.setSpeedPID(0.8, 0.3, 0);
}

double Chassis::rpmToTicksPerSecond(double RPM) {
    return RPM * 8096 * 19 / 60.0;
}

double Chassis::ticksPerSecondToRPM(double ticksPerSecond) {
    return ticksPerSecond * 60 / (8096.0 * 19.0);
}

void Chassis::setMotorPower(int index, double power) {
    switch (index) {
        case 0:
            LF.setPower(power);
            break;
        case 1:
            RF.setPower(power);
            break;
        case 2:
            LB.setPower(power);
            break;
        case 3:
            RB.setPower(power);
            break;
    }
}

void Chassis::setMotorSpeedTicksPerSecond(int index, double speed) {
    switch (index) {
        case 0:
            LF.setSpeed(speed);
            break;
        case 1:
            RF.setSpeed(speed);
            break;
        case 2:
            LB.setSpeed(speed);
            break;
        case 3:
            RB.setSpeed(speed);
            break;
    }
}

void Chassis::setMotorSpeedRPM(int index, double speed) {
    setMotorSpeedTicksPerSecond(index, rpmToTicksPerSecond(speed));
}

// Math comes from this paper: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
void Chassis::driveXYR(double xVelocityRPM, double yVelocityRPM, double rotationVelocityRPM) {
    setMotorSpeedRPM(0, 
        (xVelocityRPM + yVelocityRPM + rotationVelocityRPM)
    );
    setMotorSpeedRPM(1, 
        (xVelocityRPM - yVelocityRPM + rotationVelocityRPM)
    );
    setMotorSpeedRPM(2, 
        (-xVelocityRPM + yVelocityRPM + rotationVelocityRPM)
    );
    setMotorSpeedRPM(3, 
        (-xVelocityRPM - yVelocityRPM + rotationVelocityRPM)
    );
}

void Chassis::driveFieldRelative(double xVelocityRPM, double yVelocityRPM, double rotationVelocityRPM) {
    double angleRadians = getAngleRadians();
    double robotRelativeXVelocity = xVelocityRPM * cos(angleRadians) - yVelocityRPM * sin(angleRadians);
    double robotRelativeYVelocity = xVelocityRPM * sin(angleRadians) + yVelocityRPM * cos(angleRadians);
    driveXYR(robotRelativeXVelocity, robotRelativeYVelocity, rotationVelocityRPM);
}

void Chassis::driveAngle(double angleRadians, double speedRPM, double rotationVelocityRPM) {
    double vY = speedRPM * cos(angleRadians);
    double vX = speedRPM * sin(angleRadians);
    driveXYR(vX, vY, rotationVelocityRPM);
}

CANMotor Chassis::getMotor(int index) {
    switch (index) {
        case 0:
            return LF;
        case 1:
            return RF;
        case 2:
            return LB;
        case 3:
            return RB;
    }
}