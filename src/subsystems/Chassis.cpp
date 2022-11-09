#include "Chassis.h"

#define PI 3.14159265

#define CAN_BUS_TYPE NewCANHandler::CANBUS_1
#define MOTOR_TYPE M3508

#define kP 1
#define kI 0
#define kD 0

#define WHEEL_RADIUS_INCHES 6
#define HALF_TRACK_WIDTH_INCHES 12
#define HALF_TRACK_LENGTH_INCHE 12

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
    // LF.setSpeedPID(.743, 0.204, 0.284);
    // RF.setSpeedPID(1.073, 0.556, 0);
    // LB.setSpeedPID(1.75, 0.351, 5.63);
    // RB.setSpeedPID(1.081, 0.247, 0.386);
}

void Chassis::periodic() {
    CANMotor::tick();
}

double Chassis::rpmToTicksPerSecond(double RPM) {
    return RPM * 8096 * 19 / 60.0;
}

double Chassis::ticksPerSecondToRPM(double ticksPerSecond) {
    return ticksPerSecond * 60 / (8096.0 * 19.0);
}

double Chassis::rpmToInchesPerSecond(double RPM) {
    return RPM * 2 * PI * WHEEL_RADIUS_INCHES / 60.0;
}

double Chassis::inchesPerSecondToRPM(double inchesPerSecond) {
    return inchesPerSecond / (2.0 * PI * WHEEL_RADIUS_INCHES) * 60;
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
    double angularComponent = rotationVelocityRPM;//(HALF_TRACK_LENGTH_INCHE + HALF_TRACK_WIDTH_INCHES) * rotationVelocityRPM;
    setMotorSpeedRPM(0, 
        (xVelocityRPM + yVelocityRPM + angularComponent)
    );
    setMotorSpeedRPM(1, 
        (xVelocityRPM - yVelocityRPM + angularComponent)
    );
    setMotorSpeedRPM(2, 
        (-xVelocityRPM + yVelocityRPM + angularComponent)
    );
    setMotorSpeedRPM(3, 
        (-xVelocityRPM - yVelocityRPM + angularComponent)
    );
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