#include "Chassis.h"

#define SECONDS_PER_MINUTE 60
#define TICKS_PER_ROTATION 8096.0
#define M3508_GEAR_RATIO 19.0
#define PI 3.14159265

#define CAN_BUS_TYPE CANHandler::CANBUS_1
#define MOTOR_TYPE M3508
#define INPUT_THRESHOLD 0.01

Chassis::Chassis(short lfId, short rfId, short lbId, short rbId) : LF(lfId, CAN_BUS_TYPE, MOTOR_TYPE), RF(rfId, CAN_BUS_TYPE, MOTOR_TYPE),
                                                                   LB(lbId, CAN_BUS_TYPE, MOTOR_TYPE), RB(rbId, CAN_BUS_TYPE, MOTOR_TYPE), i2c(I2C_SDA, I2C_SCL), imu(i2c, IMU_RESET, MODE_IMU){
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
            LF.setSpeed(speed * isInverted[0]);
            break;
        case 1:
            RF.setSpeed(speed * isInverted[1]);
            break;
        case 2:
            LB.setSpeed(speed * isInverted[2]);
            break;
        case 3:
            RB.setSpeed(speed * isInverted[3]);
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

void Chassis::driveXYRPower(double ref_chassis_power, double lX, double lY, double rX, double time_diff) {
    double scale = 1;

    LF.outCap = 1500;
    RF.outCap = 1500;
    LB.outCap = 1500;
    RB.outCap = 1500;

    PID pid(12, 0.008, 0, 0, 0);

    double power0 = lX + lY + rX;
    double power1 = lX - lY + rX;
    double power2 = 0 - lX + lY + rX;
    double power3 = 0 - lX - lY + rX;

    unsigned long time = us_ticker_read() / 1000;

    scale = abs(pid.calculate(48, ref_chassis_power, time_diff));

    if (ref_chassis_power > 40) {
        power0 /= scale;
        power1 /= scale;
        power2 /= scale;
        power3 /= scale;
    }

    setMotorPower(0, power0);
    setMotorPower(1, power1);
    setMotorPower(2, power2);
    setMotorPower(3, power3);
}

void Chassis::driveFieldRelative(double xVelocityRPM, double yVelocityRPM, double rotationVelocityRPM) {
    double robotHeading = imuAngles.yaw * PI / 180.0;
    driveOffsetAngle(xVelocityRPM, yVelocityRPM, rotationVelocityRPM, robotHeading);
}

/**
 * Drives the Chassis, compensating by a certain angle (angleOffset)
*/
void Chassis::driveOffsetAngle(double xVelocityRPM, double yVelocityRPM, double rotationVelocityRPM, double angleOffset) {
    double robotRelativeXVelocity = xVelocityRPM * cos(angleOffset) + yVelocityRPM * sin(angleOffset);
    double robotRelativeYVelocity = - xVelocityRPM * sin(angleOffset) + yVelocityRPM * cos(angleOffset);
    driveXYR(robotRelativeXVelocity, robotRelativeYVelocity, rotationVelocityRPM);
}


void Chassis::driveAngle(double angleRadians, double speedRPM, double rotationVelocityRPM) {
    double vY = speedRPM * cos(angleRadians);
    double vX = speedRPM * sin(angleRadians);
    driveXYR(vX, vY, rotationVelocityRPM);
}

void Chassis::beyblade(double xVelocityRPM, double yVelocityRPM, bool switchDirections) {
    if (switchDirections) {
        if (beybladeIncreasing) {
            if (beybladeSpeed >= MAX_BEYBLADE_SPEED) {
                beybladeIncreasing = false;
            } else {
                beybladeSpeed += 0.03;
            }
        } else {
            if (beybladeSpeed <= -MAX_BEYBLADE_SPEED) {
                beybladeIncreasing = true;
            } else {
                beybladeSpeed -= 0.03;
            }
        }
    } else if (beybladeSpeed == 0) {
        beybladeSpeed = MAX_BEYBLADE_SPEED;
    }
    driveFieldRelative(xVelocityRPM, yVelocityRPM, beybladeSpeed);
}

DJIMotor Chassis::getMotor(int index) {
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


Chassis::BrakeMode Chassis::getBrakeMode() {
    return brakeMode;
}

void Chassis::setBrakeMode(BrakeMode brakeMode) {
    this->brakeMode = brakeMode;
}

void Chassis::initializeImu() {
    imu.set_mounting_position(MT_P1);
}

void Chassis::readImu() {
    imu.get_angular_position_quat(&imuAngles);
}