//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_CHASSIS_H
#define TR_EMBEDDED_CHASSIS_H

#include <motor/DJIMotor.h>
#include <communications/CANHandler.h>
#include <peripherals/imu/BNO055.h>
#include <peripherals/oled/SSD1308.h>
#include <subsystems/ChassisKalman.h>
// #include <algorithms/WheelKalman.h>
#include <algorithms/Pose2D.h>
#include <algorithms/WheelSpeeds.h>
#include <algorithms/ChassisSpeeds.h>

#define CAN_BUS_TYPE CANHandler::CANBUS_1
#define MOTOR_TYPE M3508
#define INPUT_THRESHOLD 0.01

#define I2C_SDA PB_7
#define I2C_SCL PB_8
#define IMU_RESET PA_8

#define MAX_BEYBLADE_SPEED 1800
#define BEYBLADE_ACCELERATION 0.05

/**
 * The Chassis class is a wrapper for the DJIMotor class that allows for easy control of the chassis.
 * It also contains methods for controlling the chassis in a field relative manner, and for controlling the chassis
 * with an offset angle.
 *
 * The Chassis class also contains methods for controlling the chassis with the IMU. The IMU is used to control the
 * chassis in a field relative manner, and to control the chassis with an offset angle.
 */
class Chassis
{
public:
    /**
     * The Chassis constructor. This constructor takes in the CAN IDs of the four motors on the chassis.
     * @param lfId Left front motor CAN ID
     * @param rfId Right front motor CAN ID
     * @param lbId Left back motor CAN ID
     * @param rbId Right back motor CAN ID
     */
    Chassis(short lfId, short rfId, short lbId, short rbId, I2C *i2c);

    /**
     * The BrakeMode enum is used to set the brake mode of the chassis.
     */
    enum BrakeMode
    {
        BRAKE,
        COAST
    };

    /**
     * The driveMotors method drives each motor at a specific speed
     * @param speeds The speeds in RPM to drive each motor at
     */
    void driveMotors(WheelSpeeds speeds);

    /**
     * The driveXYR method is used to drive the chassis in a chassis relative manner.
     *
     * @param speeds The robot-relative speeds (x, y, and rotation) in RPM
     */
    void driveXYR(ChassisSpeeds speeds);

    void driveXYRPower(float chassis_power, uint16_t chassis_power_limit, double lX, double lY, double dt, bool beyblading, double &rotationalPower);

    void driveOffsetAnglePower(float chassis_power, uint16_t chassis_power_limit, ChassisSpeeds speeds, double angleOffset, int dt, double &rotationalPower);

    void driveTurretRelativePower(float chassis_power, uint16_t chassis_power_limit, ChassisSpeeds speeds, double turretAngleDegrees, int dt, double &rotationalPower);

    /**
     * The driveFieldRelative method is used to drive the chassis in a field relative manner.
     *
     * @param speeds The field-relative speeds (x, y, and rotation) in RPM
     */
    void driveFieldRelative(ChassisSpeeds speeds);

    /**
     * The driveTurretRelative method is used to drive the chassis in a turret relative manner.
     *
     * @param speeds The turret-relative speeds (x, y, and rotation) in RPM
     * @param turretAngleDegrees The CCW-positive angle of the turret in degrees
     */
    void driveTurretRelative(ChassisSpeeds speeds, double turretAngleDegrees);

    /**
     * The driveOffsetAngle method is used to drive the chassis with an offset angle.
     *
     * @param speeds The speeds (x, y, and rotation) in RPM
     * @param angleOffset The angle offset in radians
     */
    void driveOffsetAngle(ChassisSpeeds speeds, double angleOffset);

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
     * @param turretAngleDegrees The turret's angle in degrees, used to drive turret relative
     * @param switchDirections Whether or not to switch directions
     */
    void beyblade(double xVelocityRPM, double yVelocityRPM, double turretAngleDegrees, bool switchDirections);

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
     * A method that should be run every main loop to update the Chassis's estimated position
     */
    void periodic();

    /**
     * Prints the current angles of the Chassis's motors, used for debugging
     */
    void printMotorAngle();

    /**
     * Helper method to convert an angle from degrees to radians
     * @param degrees An angle measurement in degrees
     * @return The angle converted to radians
     */
    double degreesToRadians(double degrees);

    /**
     * Gets the IMU's current angle reading in degrees
     * @return The IMU's current angle reading in degrees
     */
    int getHeadingDegrees();

    /**
     * Gets the chassis's current 2D position
     * @return The chassis's current 2D position
     */
    Pose2D getPose();

        /**
     * Gets the chassis's current speeds
     * @return The chassis's current speeds
     */
    ChassisSpeeds getSpeeds();
    /**
     * A helper method to read/update the IMU.
     */
    void readImu();

    bool allMotorsConnected();

    int8_t isInverted[4];

    double prevVel;

    int testData[300][4];
    int testDataIndex = 0;

private:
    DJIMotor LF, RF, LB, RB;
    BrakeMode brakeMode;

    double beybladeSpeed;
    bool beybladeIncreasing;
    BNO055 imu;
    BNO055_ANGULAR_POSITION_typedef imuAngles;

    double rpmToTicksPerSecond(double RPM);
    double ticksPerSecondToRPM(double ticksPerSecond);
    double ticksPerSecondToInchesPerSecond(double ticksPerSecond);
    double rpmToInchesPerSecond(double RPM);

    WheelSpeeds chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds);

    void setMotorPower(int index, double power);
    void setMotorSpeedRPM(int index, double speed);
    void setMotorSpeedTicksPerSecond(int index, double speed);

    double getMotorSpeedRPM(int index);

    ChassisKalman chassisKalman;
    double testAngle;
    int lastTimeMs;

    short lfId;
    short rfId;
    short lbId;
    short rbId;
};

#endif // TR_EMBEDDED_CHASSIS_H
