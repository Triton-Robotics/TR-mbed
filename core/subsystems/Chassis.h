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

/**
 * The Chassis class is a wrapper for the DJIMotor class that allows for easy control of the chassis.
 * It also contains methods for controlling the chassis in a field relative manner, and for controlling the chassis
 * with an offset angle.
 *
 * The Chassis class also contains methods for controlling the chassis with the IMU. The IMU is used to control the
 * chassis in a field relative manner, and to control the chassis with an offset angle.
 */
class Chassis {
public:
    /**
     * The Chassis constructor. This constructor takes in the CAN IDs of the four motors on the chassis.
     * @param lfId Left front motor CAN ID
     * @param rfId Right front motor CAN ID
     * @param lbId Left back motor CAN ID
     * @param rbId Right back motor CAN ID
     */
    Chassis(short lfId, short rfId, short lbId, short rbId);

    /**
     * The BrakeMode enum is used to set the brake mode of the chassis.
     */
    enum BrakeMode {
        BRAKE,
        COAST
    };

    /**
     * The driveXYR method is used to drive the chassis in a chassis relative manner.
     *
     * @param yVelocityRPM Robot velocity in the y direction in RPM
     * @param xVelocityRPM Robot velocity in the x direction in RPM
     * @param rotationVelocityRPM robot rotation velocity in RPM
     */
    void driveXYR(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);

    /**
     * The driveFieldRelative method is used to drive the chassis in a field relative manner.
     *
     * @param yVelocityRPM Robot velocity in the y direction in RPM
     * @param xVelocityRPM Robot velocity in the x direction in RPM
     * @param rotationVelocityRPM Robot rotation velocity in RPM
     */
    void driveFieldRelative(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM);

    /**
     * The driveOffsetAngle method is used to drive the chassis with an offset angle.
     *
     * @param yVelocityRPM Robot velocity in the y direction in RPM
     * @param xVelocityRPM Robot velocity in the x direction in RPM
     * @param rotationVelocityRPM Robot rotation velocity in RPM
     * @param angleOffset The angle offset in radians
     */
    void driveOffsetAngle(double yVelocityRPM, double xVelocityRPM, double rotationVelocityRPM, double angleOffset);

    /**
     * The driveAngle method is used to drive the chassis with an absolute angle.
     *
     * @param angleRadians Robot angle in radians
     * @param speedRPM Speed in RPM
     * @param rotationVelcotiyRPM Robot rotation velocity in RPM
     */
    void driveAngle(double angleRadians, double speedRPM, double rotationVelcotiyRPM);

    /**
     * The beyblade method is used to spin the chassis while moving.
     *
     * @param xVelocityRPM The x velocity in RPM
     * @param yVelocityRPM The y velocity in RPM
     * @param switchDirections Whether or not to switch directions
     */
    void beyblade(double xVelocityRPM, double yVelocityRPM, bool switchDirections);

    /**
     * A helper method to find a DJIMotor object from an index.
     *
     * @param index The index of the motor
     * @return a DJIMotor object
     */
    DJIMotor getMotor(int index);

    /**
     * A helper method to find the brake mode of the chassis.
     *
     * @return An enum representing the brake mode of the chassis
     */
    BrakeMode getBrakeMode();

    /**
     * A helper method to set the brake mode of the chassis.
     *
     * @param brakeMode An enum representing the brake mode of the chassis
     */
    void setBrakeMode(BrakeMode brakeMode);

    /**
     * A helper method to initialize the IMU.
     */
    void initializeImu();

    /**
     * A helper method to read/update the IMU.
     */
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
