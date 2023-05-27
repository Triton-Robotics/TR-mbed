#include "Chassis.h"
#include <math.h>

Chassis::Chassis(short lfId, short rfId, short lbId, short rbId) : LF(lfId, CAN_BUS_TYPE, MOTOR_TYPE), RF(rfId, CAN_BUS_TYPE, MOTOR_TYPE),
LB(lbId, CAN_BUS_TYPE, MOTOR_TYPE), RB(rbId, CAN_BUS_TYPE, MOTOR_TYPE), i2c(I2C_SDA, I2C_SCL), imu(i2c, IMU_RESET, MODE_IMU), oled(&i2c), chassisKalman() {
    printf("Chassis constructor start\n");
    LF.outCap = 16000;
    RF.outCap = 16000;
    LB.outCap = 16000;
    RB.outCap = 16000;
    LF.setSpeedPID(1.5, 0, 0);
//    LF.printAngle = true;
//    LF.setPositionPID(0.25, 0, 0.35);
//    LF.pidPosition.setIntegralCap(30000);
//    LF.useAbsEncoder = 0;

    this->lfId= lfId;
    this->rfId = rfId;
    this->lbId  = lbId;
    this->lbId = lbId;
    LF.useKalmanForPID = 1;
    RF.setSpeedPID(1.5, 0, 0);
    LB.setSpeedPID(1.5, 0, 0);
    RB.setSpeedPID(1.5, 0, 0);
    brakeMode = BRAKE;
    isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
    printf("Chassis constructor end\n");
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
    if (power > 100) {}
    power = 100;
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

double Chassis::getMotorSpeedRPM(int index) {
    return ticksPerSecondToRPM(getMotor(index).getData(VELOCITY));
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

void Chassis::driveXYRPower(double ref_chassis_power, double lX, double lY, double rX, double time_diff) {
    double scale = 1;

    PID power_pid(12, 0.008, 0, 0, 0);
    double powerLF = LF.pidSpeed.calculate(lX + lY + rX, LF.getData(VELOCITY), time_diff);
    double powerRF = RF.pidSpeed.calculate(lX - lY + rX, RF.getData(VELOCITY), time_diff);
    double powerLB = LB.pidSpeed.calculate(0 - lX + lY + rX, LB.getData(VELOCITY), time_diff);
    double powerRB = RB.pidSpeed.calculate(0 - lX - lY + rX, RB.getData(VELOCITY), time_diff);

    unsigned long time = us_ticker_read() / 1000;

    printf("REF POWER: %i\n", (int) ref_chassis_power);

    scale = abs(power_pid.calculate(48, ref_chassis_power, time_diff));

    if (ref_chassis_power > 40) {
        powerLF /= scale;
        powerRF /= scale;
        powerLB /= scale;
        powerRB /= scale;
    }

    setMotorPower(0, powerLF);
    setMotorPower(1, powerRF);
    setMotorPower(2, powerLB);
    setMotorPower(3, powerRB);
}


void Chassis::driveFieldRelative(ChassisSpeeds speeds) {
    double robotHeading = imuAngles.yaw * PI / 180.0;
    driveOffsetAngle({speeds.x, speeds.y, speeds.rotation}, robotHeading);
//    LF.setPosition((int) (speeds.y));
//printf("%i\t%i\n", (int) LF.getData(MULTITURNANGLE), (int) LF.kalman.getX(0));
}

void Chassis::driveTurretRelative(ChassisSpeeds speeds, double turretAngleDegrees) {
    double robotHeading = imuAngles.yaw * PI / 180.0;
//    printf("Turret angle: %i\n", (int) turretAngleDegrees);
    driveOffsetAngle(speeds, -turretAngleDegrees * PI / 180.0);
}


void Chassis::printMotorAngle() {
    printf("LF: %i\n", (int) LF.getData(MULTITURNANGLE));
    printf("LB: %i\n", (int) LB.getData(MULTITURNANGLE));
    printf("RF: %i\n", (int) RF.getData(MULTITURNANGLE));
    printf("RB: %i\n", (int) RB.getData(MULTITURNANGLE));
}

/**`
 * Drives the Chassis, compensating by a certain angle (angleOffset)
*/
void Chassis::driveOffsetAngle(ChassisSpeeds speeds, double angleOffset) {
    double robotRelativeXVelocity = speeds.x * cos(angleOffset) + speeds.y * sin(angleOffset);
    double robotRelativeYVelocity = - speeds.x * sin(angleOffset) + speeds.y * cos(angleOffset);
    driveXYR({robotRelativeXVelocity, robotRelativeYVelocity, speeds.rotation});
}

void Chassis::driveFieldRelativePower(double ref_chassis_power, double time_diff, double xVelocityRPM, double yVelocityRPM, double rotationVelocityRPM) {
    double robotHeading = imuAngles.yaw * PI / 180.0;
    driveOffsetAnglePower(ref_chassis_power, time_diff, xVelocityRPM, yVelocityRPM, rotationVelocityRPM, robotHeading);
}

/**
 * Drives the Chassis, compensating by a certain angle (angleOffset)
*/
void Chassis::driveOffsetAnglePower(double ref_chassis_power, double time_diff, double xVelocityRPM, double yVelocityRPM, double rotationVelocityRPM, double angleOffset) {
    double robotRelativeXVelocity = xVelocityRPM * cos(angleOffset) + yVelocityRPM * sin(angleOffset);
    double robotRelativeYVelocity = - xVelocityRPM * sin(angleOffset) + yVelocityRPM * cos(angleOffset);
    driveXYRPower(ref_chassis_power, robotRelativeXVelocity, robotRelativeYVelocity, rotationVelocityRPM, time_diff);
}

void Chassis::driveAngle(double angleRadians, double speedRPM, double rotationVelocityRPM) {
    double vY = speedRPM * cos(angleRadians);
    double vX = speedRPM * sin(angleRadians);
    driveXYR({vX, vY, rotationVelocityRPM});
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

WheelSpeeds Chassis::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds) {
    return {
            chassisSpeeds.x + chassisSpeeds.y + chassisSpeeds.rotation,
            chassisSpeeds.x - chassisSpeeds.y + chassisSpeeds.rotation,
            -chassisSpeeds.x + chassisSpeeds.y + chassisSpeeds.rotation,
            -chassisSpeeds.x - chassisSpeeds.y + chassisSpeeds.rotation
    };
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
//    printMotorAngle();
//    printf("POW: %i\n", (int) LF.powerOut);
    double z[5] = {0, 0, 0, 0, 0};
    z[0] = rpmToInchesPerSecond(LF.getData(VELOCITY));
    z[1] = rpmToInchesPerSecond(RF.getData(VELOCITY));
    z[2] = rpmToInchesPerSecond(LB.getData(VELOCITY));
    z[3] = rpmToInchesPerSecond(RB.getData(VELOCITY));

    oled.fillDisplay();
//    printf("Write result: %i\n", oled.writeChar('t'));

    z[4] = imuAngles.yaw;

//    printf("Yaw (IMU): %i\n", (int) z[4]);

    int currTime = us_ticker_read();
    if (lastTimeMs != 0) {
        chassisKalman.setDt((currTime - lastTimeMs) / 1000000.0);
    }
    lastTimeMs = currTime;
    chassisKalman.step(z);


//    printf("%i\t%i\t%i\n", (int) chassisKalman.getX(0), (int) chassisKalman.getX(2), (int) chassisKalman.getX(4));

//    double z[2] = { 0, 0 };
//    double angle = wheelKalman.getX(0);
//    z[1] = rpmToTicksPerSecond(LF.getData(VELOCITY)) * M3508_GEAR_RATIO;
//    double measured = LF.getData(ANGLE);
//    int MODULUS = 8192;
//    z[0] = (measured + angle - ((int) angle) % MODULUS);
    int power =  LF.getPowerOut() - 300;

//    int diff = (int) (z[1] - prevVel);
//    if (diff != 0) {
//        printf("%i\t%i\n",  LF.getPowerOut(), (int) (z[1] - prevVel));
//    }
//    prevVel = z[1];
//printf("Power: %i\n", LF.getPowerOut());


//    if (testDataIndex == 300) {
//
//    } else {
//        testData[testDataIndex][0] = us_ticker_read() / 1000;
//        testData[testDataIndex][1] = (int) (wheelKalman.getX(0) / M3508_GEAR_RATIO);
//        testData[testDataIndex][2] = (int) (LF.getData(MULTITURNANGLE) / M3508_GEAR_RATIO);
//        testData[testDataIndex][3] = (int) (z[1] / M3508_GEAR_RATIO);
//        testDataIndex++;
//    }

//    printf("%i\t%i\n", (int) z[2], (int) z[0]);


//    printf("%i\t%i\n", (int) (wheelKalman.getX(0) / M3508_GEAR_RATIO), (int) (z[0] / M3508_GEAR_RATIO));
//    printf("%i\t%i\n", (int) (wheelKalman.getX(1)), (int) ((z[1])));

//    printf("Delta angle over delta t: %i\n", (int) ((LF.getData(ANGLE) - prevVel) / wheelKalman.dt));
//    prevVel = measured;
    //    printf("Measured: %i \t Estimate: %i\n", (int) measured, (int) (angle / M3508_GEAR_RATIO));
//    printf("Velo measure: %i   estimate: %i\n", (int) z[1], (int) wheelKalman.getX(1));
    //    printf("Kalman accel: %i\n", (int) wheelKalman.getX(1));
//    printf("Multiturn: %i\n", (int) (LF.getData(MULTITURNANGLE) / M3508_GEAR_RATIO));
//printf("TORQUE: %i\n", (int) LF.getData(TORQUE));
//int torque = (int) LF.getData(TORQUE);
//int result[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//int16ToBitArray(torque, result);
//printArray(result, 16);
//printf("%i\t%i\n", (int) LF.getPowerOut(), torque);
//for (int i = 0; i < 10; i++) {
//    printf("Bit %i: %i\n", i, (torque >> i) % 2);
//}
}

void Chassis::readImu() {
//    double lastAngle = imuAngles.yaw;
//printf("Reading imu: \n");
    imu.get_angular_position_quat(&imuAngles);
//    double curAngle = imuAngles.yaw;
//    double deltaAngle = curAngle - lastAngle;
//    if (deltaAngle < -300) {
//        imuAngles.yaw += 360;
//    }
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

double Chassis::degreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

int Chassis::getHeadingDegrees() {
    return (int) imu.multiturnYaw;
}


Pose2D Chassis::getPose() {
    return {
        chassisKalman.getX(0),
        chassisKalman.getX(2),
        Chassis::degreesToRadians(chassisKalman.getX(4))
    };
}

ChassisSpeeds Chassis::getSpeeds() {
    return {
        chassisKalman.getX(1),
        chassisKalman.getX(3),
        chassisKalman.getX(4)
    };
}


bool Chassis::allMotorsConnected() {
    return (
            (DJIMotor::isMotorConnected(lfId, CAN_BUS_TYPE, MOTOR_TYPE)) &&
                    (DJIMotor::isMotorConnected(rfId, CAN_BUS_TYPE, MOTOR_TYPE)) &&
                    (DJIMotor::isMotorConnected(lbId, CAN_BUS_TYPE, MOTOR_TYPE)) &&
                    (DJIMotor::isMotorConnected(rbId, CAN_BUS_TYPE, MOTOR_TYPE))

    );
}