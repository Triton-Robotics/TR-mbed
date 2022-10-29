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

Chassis::Chassis(): LF(4, CAN_BUS_TYPE, MOTOR_TYPE), RF(2, CAN_BUS_TYPE, MOTOR_TYPE), 
        LB(1, CAN_BUS_TYPE, MOTOR_TYPE), RB(3, CAN_BUS_TYPE, MOTOR_TYPE) {
    motors[0] = LF;
    motors[1] = RF;
    motors[2] = LB;
    motors[3] = RB;
    for (int i = 0; i < 4; i++) {
        motors[i].setSpeedPID(kP, kI, kD);
        motors[i].outCap = 16000;   
    }
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
    motors[index].setPower(power);
}

void Chassis::setMotorSpeedRPM(int index, double speed) {
    motors[index].setSpeed(rpmToTicksPerSecond(speed));
}

void Chassis::setMotorSpeedInchesPerSecond(int index, double speed) {
    setMotorSpeedRPM(index, inchesPerSecondToRPM(speed));
}

void Chassis::tankDrive(double leftVelocityRPM, double rightVelocityRPM) {
    setMotorSpeedRPM(0, leftVelocityRPM);
    setMotorSpeedRPM(2, leftVelocityRPM);
    setMotorSpeedRPM(1, rightVelocityRPM);
    setMotorSpeedRPM(3, rightVelocityRPM);
}

// Math comes from this paper: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
void Chassis::drive1(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM) {
    double angularComponent = (HALF_TRACK_LENGTH_INCHE + HALF_TRACK_WIDTH_INCHES) * rotationVelocityRPM;
    setMotorSpeedInchesPerSecond(0, 
        (xVelocityRPM - yVelocityRPM - angularComponent) / WHEEL_RADIUS_INCHES
    );
    setMotorSpeedInchesPerSecond(1, 
        (xVelocityRPM + yVelocityRPM + angularComponent) / WHEEL_RADIUS_INCHES
    );
    setMotorSpeedInchesPerSecond(2, 
        (xVelocityRPM + yVelocityRPM - angularComponent) / WHEEL_RADIUS_INCHES
    );
    setMotorSpeedInchesPerSecond(3, 
        (xVelocityRPM - yVelocityRPM + angularComponent) / WHEEL_RADIUS_INCHES
    );
}

void Chassis::drive2(double angleRadians, double speedRPM, double rotationVelcotiyRPM) {
    double vX = speedRPM * cos(angleRadians);
    double vY = speedRPM * sin(angleRadians);
    drive1(vY, vX, rotationVelcotiyRPM);
}

CANMotor Chassis::getMotor(int index) {
    return motors[index];
}