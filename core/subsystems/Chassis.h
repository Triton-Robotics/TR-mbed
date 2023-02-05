//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_CHASSIS_H
#define TR_EMBEDDED_CHASSIS_H

#include <motor/DJIMotor.h>
#include <communications/CANHandler.h>
#include <peripherals/imu/BNO055.h>

#define I2C_SDA PB_9
#define I2C_SCL PB_8
#define IMU_RESET PA_8

#define MAX_BEYBLADE_SPEED 1.5
#define BEYBLADE_ACCELERATION 0.05


class Chassis {
public:
    Chassis(short lfId, short rfId, short lbId, short rbId);

    enum BrakeMode {
        BRAKE,
        COAST
    };

    void driveXYR(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
    void driveFieldRelative(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);
    void driveOffsetAngle(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM, double angleOffset);
    void driveAngle(double angleRadians, double speedRPM, double rotationVelcotiyRPM);
    void beyblade(double xVelocityRPM, double yVelocityRPM, bool switchDirections);
    DJIMotor getMotor(int index);

    BrakeMode getBrakeMode();
    void setBrakeMode(BrakeMode brakeMode);
    void initializeImu();
    void readImu();

    int8_t isInverted[4];

private:
    DJIMotor LF, RF, LB, RB;
    BrakeMode brakeMode;

    double beybladeSpeed;
    bool beybladeIncreasing;
    I2C i2c;
    BNO055 imu;
    BNO055_ANGULAR_POSITION_typedef imuAngles;

    double rpmToTicksPerSecond(double RPM);
    double ticksPerSecondToRPM(double ticksPerSecond);

    void setMotorPower(int index, double power);
    void setMotorSpeedRPM(int index, double speed);
    void setMotorSpeedTicksPerSecond(int index, double speed);
};

#endif //TR_EMBEDDED_CHASSIS_H
