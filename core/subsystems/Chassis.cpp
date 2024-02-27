#include "Chassis.h"
#include <cmath>

Chassis::Chassis(short lfId, short rfId, short lbId, short rbId, I2C *i2c) : LF(lfId, CAN_BUS_TYPE, MOTOR_TYPE), RF(rfId, CAN_BUS_TYPE, MOTOR_TYPE),
LB(lbId, CAN_BUS_TYPE, MOTOR_TYPE), RB(rbId, CAN_BUS_TYPE, MOTOR_TYPE),  imu(*i2c, IMU_RESET, MODE_IMU), chassisKalman() {
    LF.outputCap = 16000;
    RF.outputCap = 16000;
    LB.outputCap = 16000;
    RB.outputCap = 16000;
    LF.setSpeedPID(1.5, 0, 0);

    this->lfId = lfId;
    this->rfId = rfId;
    this->lbId = lbId;
    this->rbId = rbId;

    RF.setSpeedPID(1.5, 0, 0);
    LB.setSpeedPID(1.5, 0, 0);
    RB.setSpeedPID(1.5, 0, 0);
    brakeMode = BRAKE;
    isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}

double Chassis::rpmToTicksPerSecond(double RPM) {
    return RPM * TICKS_PER_ROTATION / (M3508_GEAR_RATIO * SECONDS_PER_MINUTE);
}

double Chassis::ticksPerSecondToRPM(double ticksPerSecond) {
    return ticksPerSecond * SECONDS_PER_MINUTE * M3508_GEAR_RATIO / TICKS_PER_ROTATION;
}

double Chassis::ticksPerSecondToInchesPerSecond(double ticksPerSecond) {
    double result = ticksPerSecond / (TICKS_PER_ROTATION * M3508_GEAR_RATIO) * WHEEL_DIAMETER_INCHES * PI;
    return result;
}

double Chassis::rpmToInchesPerSecond(double RPM) {
    return RPM * 1.41 / (M3508_GEAR_RATIO * SECONDS_PER_MINUTE) * WHEEL_DIAMETER_INCHES * PI;
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
    setMotorSpeedTicksPerSecond(index, speed);
}

void Chassis::driveMotors(WheelSpeeds speeds) {
    setMotorSpeedRPM(0, speeds.LF);
    setMotorSpeedRPM(1, speeds.RF);
    setMotorSpeedRPM(2, speeds.LB);
    setMotorSpeedRPM(3, speeds.RB);
}

// Math comes from this paper: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
void Chassis::driveXYR(ChassisSpeeds speeds) {
    driveMotors(Chassis::chassisSpeedsToWheelSpeeds(speeds));
}

void Chassis::driveXYR1(int speedX, int speedY, int speedR ) {
    setMotorSpeedRPM(0, speedX + speedY + speedR);
    setMotorSpeedRPM(1, speedX - speedY + speedR);
    setMotorSpeedRPM(2, -speedX + speedY+ speedR);
    setMotorSpeedRPM(3, -speedX - speedY + speedR);
}

void Chassis::driveXYRPower(float chassis_power, uint16_t chassis_power_limit, double lX, double lY, double dt, bool beyblading, double &rotationalPower) {

    if (beyblading) {
        rotationalPower = -(chassis_power_limit) * 100;
    }else {
        rotationalPower = 0;
    }

    double scale = 1;

    PID power_pid(12, 0.01, 10, 1, 0);
    double powerLF = LF.pidSpeed.calculate(lX + lY, LF.getData(VELOCITY), dt) + rotationalPower;
    double powerRF = RF.pidSpeed.calculate(lX - lY, RF.getData(VELOCITY), dt) + rotationalPower;
    double powerLB = LB.pidSpeed.calculate(-lX + lY, LB.getData(VELOCITY), dt) + rotationalPower;
    double powerRB = RB.pidSpeed.calculate(-lX - lY, RB.getData(VELOCITY), dt) + rotationalPower;

    scale = abs(power_pid.calculate((float)chassis_power_limit - 15, chassis_power, dt));

    if (chassis_power > (chassis_power_limit - 10)) {
        powerLF /= scale;
        powerRF /= scale;
        powerLB /= scale;
        powerRB /= scale;
    }

    LF.setPower(powerLF);
    RF.setPower(powerRF);
    LB.setPower(powerLB);
    RB.setPower(powerRB);
}

//updated this in a new function below
void Chassis::driveFieldRelative(ChassisSpeeds speeds) {
    double robotHeading = imuAngles.yaw * PI / 180.0;
    driveOffsetAngle({speeds.x, speeds.y, speeds.rotation}, robotHeading);
}

void Chassis::driveTurretRelative(ChassisSpeeds speeds, double turretAngleDegrees) {
    //double robotHeading = imuAngles.yaw * PI / 180.0;
    driveOffsetAngle(speeds, -turretAngleDegrees * PI / 180.0);
}

void Chassis::driveTurretRelativePower(float chassis_power, uint16_t chassis_power_limit, ChassisSpeeds speeds, double turretAngleDegrees, int dt,  double &rotationalPower) {
    driveOffsetAnglePower(chassis_power, chassis_power_limit, speeds, -turretAngleDegrees * PI / 180.0, dt,rotationalPower);
}

/**`
 * Drives the Chassis, compensating by a certain angle (angleOffset)
*/
void Chassis::driveOffsetAngle(ChassisSpeeds speeds, double angleOffset) {
    double robotRelativeXVelocity = speeds.x * cos(angleOffset) + speeds.y * sin(angleOffset);
    double robotRelativeYVelocity = - speeds.x * sin(angleOffset) + speeds.y * cos(angleOffset);
    driveXYR({robotRelativeXVelocity, robotRelativeYVelocity, speeds.rotation});
}

//new function
void Chassis::driveOffsetAngle1(int speedX, int speedY, int speedR, double angleOffset) {
    double robotRelativeXVelocity = speedX * cos(angleOffset) + speedY * sin(angleOffset);
    double robotRelativeYVelocity = - speedX * sin(angleOffset) + speedY * cos(angleOffset);
    driveXYR({robotRelativeXVelocity, robotRelativeYVelocity, speedR});
}

void Chassis::driveOffsetAnglePower(float chassis_power, uint16_t chassis_power_limit, ChassisSpeeds speeds, double angleOffset, int dt,  double &rotationalPower) {
    double robotRelativeXVelocity = speeds.x * cos(angleOffset) + speeds.y * sin(angleOffset);
    double robotRelativeYVelocity = - speeds.x * sin(angleOffset) + speeds.y * cos(angleOffset);
    driveXYRPower(chassis_power, chassis_power_limit, robotRelativeXVelocity, robotRelativeYVelocity, dt, bool(speeds.rotation), rotationalPower);
}

void Chassis::driveAngle(double angleRadians, double speedRPM, double rotationVelocityRPM) {
    double vY = speedRPM * cos(angleRadians);
    double vX = speedRPM * sin(angleRadians);
    driveXYR({vX, vY, rotationVelocityRPM});
}

void Chassis::driveAngle1(double angleDegrees, double speedRPM, double rotationVelocityRPM) {
    driveAngle(degreesToRadians(angleDegrees), speedRPM, rotationVelocityRPM);
}


void Chassis::beyblade(double xVelocityRPM, double yVelocityRPM, double turretAngleDegrees, bool switchDirections) {
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
    driveTurretRelative({
        xVelocityRPM,
        yVelocityRPM,
        beybladeSpeed
    }, turretAngleDegrees);
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

void Chassis::initializeImu() {
    imu.set_mounting_position(MT_P1);
}

void Chassis::periodic() {
    double z[5] = {0, 0, 0, 0, 0};
    z[0] = rpmToInchesPerSecond(LF.getData(VELOCITY));
    z[1] = rpmToInchesPerSecond(RF.getData(VELOCITY));
    z[2] = rpmToInchesPerSecond(LB.getData(VELOCITY));
    z[3] = rpmToInchesPerSecond(RB.getData(VELOCITY));

    z[4] = imuAngles.yaw;

    int currTime = us_ticker_read();
    if (lastTimeMs != 0) {
        chassisKalman.setDt((currTime - lastTimeMs) / 1000000.0);
    }
    lastTimeMs = currTime;
    chassisKalman.step(z);


}

void Chassis::readImu() {
    imu.get_angular_position_quat(&imuAngles);
}

double Chassis::degreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

int Chassis::getHeadingDegrees() {
    return (int) imuAngles.yaw;
}


Pose2D Chassis::getPose() {
    return {
        chassisKalman.getX(0),
        chassisKalman.getX(2),
        Chassis::degreesToRadians(chassisKalman.getX(4))
    };
}