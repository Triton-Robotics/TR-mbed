#include "Chassis.h"
#include <math.h>

Chassis::Chassis(short lfId, short rfId, short lbId, short rbId) : LF(lfId, CAN_BUS_TYPE, MOTOR_TYPE), RF(rfId, CAN_BUS_TYPE, MOTOR_TYPE),
                                                                   LB(lbId, CAN_BUS_TYPE, MOTOR_TYPE), RB(rbId, CAN_BUS_TYPE, MOTOR_TYPE), i2c(I2C_SDA, I2C_SCL), imu(i2c, IMU_RESET, MODE_IMU), chassisKalman() {
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

double Chassis::ticksPerSecondToInchesPerSecond(double ticksPerSecond) {
    double result = ticksPerSecond / (TICKS_PER_ROTATION * M3508_GEAR_RATIO) * WHEEL_DIAMETER_INCHES * PI;
    return result;
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

double Chassis::getMotorSpeedRPM(int index) {
    return ticksPerSecondToRPM(getMotor(index).getData(VELOCITY));
}

// Math comes from this paper: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
void Chassis::driveXYR(double xVelocityRPM, double yVelocityRPM, double rotationVelocityRPM) {
    double squareRoot = sqrt(xVelocityRPM * xVelocityRPM + yVelocityRPM * yVelocityRPM);
    squareRoot *= 4;
    if (squareRoot > 1) {
        rotationVelocityRPM /= squareRoot;
    }
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

void Chassis::periodic() {
    double z[5] = {0, 0, 0, 0, 0};
    z[0] = ticksPerSecondToInchesPerSecond(LF.getData(VELOCITY));
    z[1] = ticksPerSecondToInchesPerSecond(RF.getData(VELOCITY));
    z[2] = ticksPerSecondToInchesPerSecond(LB.getData(VELOCITY));
    z[3] = ticksPerSecondToInchesPerSecond(RB.getData(VELOCITY));
    z[4] = imuAngles.yaw;
    int currTime = us_ticker_read() / 1000;
    if (lastTimeMs != 0) {
        chassisKalman.setDt((currTime - lastTimeMs) / 1000.0);
    }
    lastTimeMs = currTime;
    chassisKalman.step(z);
//    printf("TMP: %i\n", (int) (1000 * x));
//    printf("Z: %i %i %i %i\n", (int) (100 * z[0]), (int) (100 * z[1]), (int) z[2], (int) z[3]);

    printf("X pos: %i\n", (int) (1000 * chassisKalman.getX(0)));
}

void Chassis::readImu() {
    imu.get_angular_position_quat(&imuAngles);
//    imuAngles.yaw += (rand() % 20 - 10);

//

//    int currTime = us_ticker_read() / 1000;
//    if (lastTimeMs != 0) {
//        testAngle += LF.getData(VELOCITY) * (currTime - lastTimeMs) / ((double) TICKS_PER_ROTATION * M3508_GEAR_RATIO);
//    }
//    lastTimeMs = currTime;
//
//    printf("Dist: %i\n", (int) testAngle);

//    BNO055_VECTOR_TypeDef imuGyro;
//    imu.get_gyro(&imuGyro);
//    printf("XYZ: %i\n", (int) imuGyro.x);

//    printf("Measurement: %i Prediction: %i\n", (int) imuAngles.yaw, (int) imuKalman.getX(0));
}