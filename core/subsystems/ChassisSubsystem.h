#ifndef TR_EMBEDDED_CHASSIS_SUBSYSTEM_H
#define TR_EMBEDDED_CHASSIS_SUBSYSTEM_H

#include "mbed.h"
#include "../util/peripherals/imu/BNO055.h"

#include <motor/DJIMotor.h>
#include <communications/CANHandler.h>
#include <peripherals/imu/BNO055.h>
#include <peripherals/oled/SSD1308.h>
// #include <subsystems/ChassisKalman.h>
// #include <algorithms/WheelKalman.h>
#include <algorithms/Pose2D.h>
// #include <algorithms/WheelSpeeds.h>
// #include <algorithms/ChassisSpeeds.h>
#include "algorithms/eigen-3.4.0/Eigen/QR"

#define CAN_BUS_TYPE CANHandler::CANBUS_1
#define MOTOR_TYPE M3508
#define M3508_POST_MAX_RPM 469
#define INPUT_THRESHOLD 0.01

#define I2C_SDA PB_7
#define I2C_SCL PB_8
#define IMU_RESET PA_8

#define PI 3.14159265
#define SECONDS_PER_MINUTE 60
#define TICKS_PER_ROTATION 8192.0
#define WHEEL_DIAMETER_METERS 4.75 * 0.0254

#define MAX_BEYBLADE_SPEED 1800
#define BEYBLADE_ACCELERATION 0.05

struct OmniKinematics
{
    double r1x, r1y, r2x, r2y, r3x, r3y, r4x, r4y;
};

struct WheelSpeeds
{
    double LF;
    double RF;
    double LB;
    double RB;

    void operator*=(double scalar)
    {
        LF *= scalar;
        RF *= scalar;
        LB *= scalar;
        RB *= scalar;
    }

    char *to_string()
    {
        char buffer[56];
        sprintf(buffer, "LF: %4.2f RF: %4.2f LB: %4.2f RB: %4.2f\0", LF, RF, LB, RB);
        return buffer;
    }
};

struct ChassisSpeeds
{
    double vX;
    double vY;
    double vOmega;
};

struct OmniKinematicsLimits
{
    double max_Vel;
    double max_vOmega;
};

/**
 * The Chassis class is a wrapper for the DJIMotor class that allows for easy control of the chassis.
 * It also contains methods for controlling the chassis in a field relative manner, and for controlling the chassis
 * with an offset angle.
 *
 * The Chassis class also contains methods for controlling the chassis with the IMU. The IMU is used to control the
 * chassis in a field relative manner, and to control the chassis with an offset angle.
 */
class ChassisSubsystem
{
public:
    /**
     * The Chassis constructor. This constructor takes in the CAN IDs of the four motors on the chassis.
     * @param lfId Left front motor CAN ID
     * @param rfId Right front motor CAN ID
     * @param lbId Left back motor CAN ID
     * @param rbId Right back motor CAN ID
     */
    ChassisSubsystem(short lfId, short rfId, short lbId, short rbId, BNO055 &imu, double radius);

    /**
     * The BrakeMode enum is used to set the brake mode of the chassis.
     */
    enum BrakeMode
    {
        BRAKE,
        COAST
    };

    enum MotorLocation
    {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    };

    enum SPEED_UNIT
    {
        RAD_PER_SECOND,
        RPM,
        METER_PER_SECOND,
    };

    enum DRIVE_MODE
    {
        YAW_ORIENTED,
        ROBOT_ORIENTED
    };

    /**
     * Gets the chassis's current WheelSpeeds
     * @return The chassis's current WheelSpeeds
     */
    WheelSpeeds getWheelSpeeds() const;

    /**
     * The setWheelSpeeds method drives each motor at a specific speed
     * @param wheelSpeeds The speeds in RPM to drive each motor at
     */
    void setWheelSpeeds(WheelSpeeds wheelSpeeds);

    /**
     * The normalizeWheelSpeeds method normalizes each motor with respect to m_OmniKinematicsLimits.max_Vel
     * @param wheelSpeeds The speeds in m/s to drive each motor at
     */
    WheelSpeeds normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const;

    /**
     * The driveMotors method drives each motor at a specific speed
     * @param wheelPower The speeds in [-1, 1] to drive each motor at
     */
    void setWheelPower(WheelSpeeds wheelPower);

    /**
     * Gets the chassis's current ChassisSpeeds
     * @return The chassis's current ChassisSpeeds
     */
    ChassisSpeeds getChassisSpeeds() const;

    /**
     * The setChassisSpeeds method is used to drive the chassis in a chassis relative manner.
     *
     * @param desiredChassisSpeeds The robot-relative speeds (vX, vY, and vOmega) in RPM
     */
    void setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode = ROBOT_ORIENTED);

    /**
     * 
     * 
     * 
    */
   
    void setChassisSpeedsPWR(double fwd, double strafe);

    /**
     * The setChassisPower method is used to drive the chassis in a chassis relative manner.
     *
     * @param desiredChassisPower The robot-relative power (vX, vY, and vOmega) in [-1, 1]
     */
    void setChassisPower(ChassisSpeeds desiredChassisPower);

    /**
     * The rotateChassisSpeed method
     *
     * @param speeds The relative speeds (vX, vY, and vOmega) in m/s
     * @param yawCurrent The CCW-positive angle in degrees
     */
    ChassisSpeeds rotateChassisSpeed(ChassisSpeeds speeds, double yawCurrent);

    /**
     * A helper method to find a DJIMotor object from an index.
     *
     * @param location The MotorLocation of the motor (LF, RF, LB, RB)
     * @return a DJIMotor object
     */
    DJIMotor &getMotor(MotorLocation location);

    /**
     * Sets the P, I, and D control parameters
     * @param location The MotorLocation of the motor (LF, RF, LB, RB)
     * @param kP The new P (proportional) parameter
     * @param kI The new I (integral) parameter
     * @param kD The new D (derivative) parameter
     */
    void setMotorSpeedPID(MotorLocation location, float kP, float kI, float kD);

    /**
     * Sets the IntegralCap for the SpeedPID
     * @param location The MotorLocation of the motor (LF, RF, LB, RB)
     * @param cap The maximum integral that can be achieved, above which the integral will be capped
     */
    void setSpeedIntegralCap(MotorLocation location, double cap);

    /**
     * Sets the Feedforward for the SpeedPID
     * @param location The MotorLocation of the motor (LF, RF, LB, RB)
     * @param FF The arbitrary Feedforward value ranges from [-1, 1]
     */
    void setSpeedFeedforward(MotorLocation location, double FF);

    /**
     * Sets the Friction Feedforward compensation for the SpeedPID
     * @param Ks The arbitrary Friction Feedforward Gain in [-1, 1]
     */
    void setSpeedFF_Ks(double Ks);

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
     * @param radians An angle measurement in radians
     * @return The angle converted to degree
     */
    static double radiansToDegrees(double radians);

    /**
     * Helper method to convert an angle from radians to degrees
     * @param degrees An angle measurement in degrees
     * @return The angle converted to radians
     */
    static double degreesToRadians(double degrees);

    /**
     * Gets the IMU's current angle reading in degrees
     * @return The IMU's current angle reading in degrees
     */
    int getHeadingDegrees() const;

    /**
     * Gets the chassis's current 2D position
     * @return The chassis's current 2D position
     */
    Pose2D getPose();

    double getMotorSpeed(MotorLocation location, SPEED_UNIT unit);

    /**
     * A helper method to read/update the IMU.
     */
    void readImu();

    bool allMotorsConnected();

    void setYawReference(DJIMotor *motor, double initial_offset_ticks);

    ChassisSpeeds desiredChassisSpeeds;
    WheelSpeeds desiredWheelSpeeds;

    OmniKinematicsLimits m_OmniKinematicsLimits;
    WheelSpeeds chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds);
    ChassisSpeeds wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds);
    char * MatrixtoString(Eigen::MatrixXd mat);

    ChassisSpeeds m_chassisSpeeds;
    WheelSpeeds m_wheelSpeeds;

    // int8_t isInverted[4];

    double prevVel;
    double yawPhase;
    int testData[300][4];
    int testDataIndex = 0;

private:
    DJIMotor LF, RF, LB, RB;
    DJIMotor *yaw = 0;
    // double yawPhase;
    BrakeMode brakeMode;

    // double beybladeSpeed;
    // bool beybladeIncreasing;
    BNO055 imu;
    BNO055_ANGULAR_POSITION_typedef imuAngles;

    static double radiansToTicks(double radians);
    static double ticksToRadians(double ticks);

    static double rpmToRadPerSecond(double RPM);
    static double radPerSecondToRPM(double radPerSecond);

    OmniKinematics m_OmniKinematics;
    void setOmniKinematics(double radius);
    // Eigen::MatrixXd wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds);

    double FF_Ks;

    void setMotorPower(MotorLocation location, double power);
    void setMotorSpeedRPM(MotorLocation location, double speed);
    // void setMotorSpeedTicksPerSecond(int index, double speed);

    double getMotorSpeedRPM(MotorLocation location);

    // ChassisKalman chassisKalman;
    double testAngle;
    int lastTimeMs;

    short lfId;
    short rfId;
    short lbId;
    short rbId;
};

#endif // TR_EMBEDDED_CHASSIS_SUBSYSTEM_H