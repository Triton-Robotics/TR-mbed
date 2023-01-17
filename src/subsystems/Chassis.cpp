#include "Chassis.h"
#include "../../constants/Chassis.h"

#define PI 3.14159265

#define CAN_BUS_TYPE NewCANHandler::CANBUS_1
#define MOTOR_TYPE M3508
#define INPUT_THRESHOLD 0.01

double getAngleRadians() {
    return PI / 2.0;
}

Chassis::Chassis(short lfId, short rfId, short lbId, short rbId) : LF(lfId, CAN_BUS_TYPE, MOTOR_TYPE), RF(rfId, CAN_BUS_TYPE, MOTOR_TYPE), 
    LB(lbId, CAN_BUS_TYPE, MOTOR_TYPE), RB(rbId, CAN_BUS_TYPE, MOTOR_TYPE) {
    LF.outCap = 16000;   
    RF.outCap = 16000;   
    LB.outCap = 16000;   
    RB.outCap = 16000;   
    LF.setSpeedPID(1.5, 0, 0);
    RF.setSpeedPID(1.5, 0, 0);
    LB.setSpeedPID(1.5, 0, 0);
    RB.setSpeedPID(1.5, 0, 0);
    brakeMode = BRAKE;
    isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}

double Chassis::rpmToTicksPerSecond(double RPM) {
    return RPM * TICKS_PER_ROTATION * M3508_GEAR_RATIO / SECONDS_PER_MINUTE;
}

double Chassis::ticksPerSecondToRPM(double ticksPerSecond) {
    return ticksPerSecond * SECONDS_PER_MINUTE / (TICKS_PER_ROTATION * M3508_GEAR_RATIO);
}

void Chassis::setMotorPower(int index, double power) {
    switch (index) {
        case 0:
            LF.setPower(power * isInverted[0]);
            break;
        case 1:
            RF.setPower(power * isInverted[1]);
            break;
        case 2:
            LB.setPower(power * isInverted[2]);
            break;
        case 3:
            RB.setPower(power * isInverted[3]);
            break;
    }
}

void Chassis::setMotorSpeedTicksPerSecond(int index, double speed) {
    if (brakeMode == COAST && speed == 0) {
        setMotorPower(index, 0);
        return;
    }
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
        default:
            return LF;
    }
}


BrakeMode Chassis::getBrakeMode() {
    return brakeMode;
}

void Chassis::setBrakeMode(BrakeMode brakeMode) {
    this->brakeMode = brakeMode;
}