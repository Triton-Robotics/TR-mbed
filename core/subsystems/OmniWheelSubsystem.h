#ifndef TR_EMBEDDED_CHASSIS_SUBSYSTEM_H
#define TR_EMBEDDED_CHASSIS_SUBSYSTEM_H

#include "mbed.h"
#include "util/peripherals/imu/IMU.h"
#include "util/communications/PwmIn.h"
#include "util/peripherals/encoder/MA4.h"

#include <cstdint>
#include <util/motor/DJIMotor.h>
#include <util/communications/CANHandler.h>
#include <util/peripherals/oled/SSD1308.h>
// #include <subsystems/ChassisKalman.h>
// #include <algorithms/WheelKalman.h>
#include <util/algorithms/PID.h>
// #include <algorithms/WheelSpeeds.h>
// #include <algorithms/ChassisSpeeds.h>

#define CAN_BUS_TYPE CANHandler::CANBUS_1
#define MOTOR_TYPE M3508
#define M3508_POST_MAX_RPM 469
#define INPUT_THRESHOLD 0.01

#define PI 3.14159265
#define SECONDS_PER_MINUTE 60
#define TICKS_PER_ROTATION 8192.0
#define WHEEL_DIAMETER_METERS 0.146
#define WHEEL_TO_CHASSIS_CENTER_LX 0.21
#define WHEEL_TO_CHASSIS_CENTER_LY 0.14

constexpr float STATIC_FRICTION_CONSTANT = 0.233924f;
constexpr float STATIC_FRICTION_CONSTANT_ALT = 0.8f;
constexpr float GRAVITY = 9.80665f;
constexpr int ACCEL_DENOM_CONSTANT = 2;

#define MAX_BEYBLADE_SPEED 1800
#define BEYBLADE_ACCELERATION 0.05
#define MAX_VEL 2.92

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
 * The OmniWheelSubsystem class is a wrapper for the DJIMotor class that allows for easy control of the chassis.
 *
 * The OmniWheelSubsystem class also contains methods for controlling the chassis with the IMU. The IMU is used to control the
 * chassis in a field-relative manner, and to control the chassis with an offset angle.
 */
class OmniWheelSubsystem
{
public:

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
        YAW_ALIGN,
        ROBOT_ORIENTED,
        ODOM_ORIENTED
    };

    enum HOLONOMIC_MODE
    {
        OMNI,
        MECANUM
    };
    
    struct Config {
        int left_front_can_id;
        int right_front_can_id;
        int left_back_can_id;
        int right_back_can_id;

        PID::config lf_pid;
        PID::config rf_pid;
        PID::config lb_pid;
        PID::config rb_pid;

        const double radius;
        const double speed_pid_ff_ks;

        float yaw_initial_offset_ticks;
        float power_limit = 60;

        HOLONOMIC_MODE chassis_type = OMNI;
    };

    /**
     * The Chassis constructor.
     */
    OmniWheelSubsystem(const Config &config, MA4 *encoder_);

    float previousRPM[4] = {0, 0, 0, 0};

    static float limitAcceleration(float desiredRPM, float previousRPM, unsigned long deltatime, float theta);

    float power_limit;

    /**
     * Gets the chassis's current WheelSpeeds
     * @return The chassis's current WheelSpeeds
     */
    WheelSpeeds getWheelSpeeds() const;

    /**
     * The setWheelSpeeds method drives each motor at a specific speed
     * @param wheelSpeeds The speeds in RPM to drive each motor at
     */
    float setWheelSpeeds(WheelSpeeds wheelSpeeds);

    /**
     * The normalizeWheelSpeeds method normalizes each motor with respect to m_OmniKinematicsLimits.max_Vel
     * @param wheelSpeeds The speeds in m/s to drive each motor at
     */
    WheelSpeeds normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const;

    /**
     * Gets the chassis's current ChassisSpeeds from odometry (only velocity state)
     * @return The chassis's current ChassisSpeeds
     */
    ChassisSpeeds getChassisSpeeds() const;

    /**
     * The setChassisSpeeds method is used to drive the chassis in a chassis relative manner.
     *
     * @param desiredChassisSpeeds The robot-relative speeds (vX, vY, and vOmega) in m/s
     */
    float setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, DRIVE_MODE mode = ROBOT_ORIENTED);

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
     * Sets the Feedforward for the SpeedPID
     * @param location The MotorLocation of the motor (LF, RF, LB, RB)
     * @param FF The arbitrary Feedforward value ranges from [-1, 1]
     */
    void setSpeedFeedforward(MotorLocation location, double FF);


    /**
     * A method that should be run every main loop to update the Chassis's estimated position
     */
    void periodic(IMU::EulerAngles *imuCurr = nullptr);

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
     * gets motor speed based on location and speed unit
     * @return motor speed in selected unit
     *
     * @param location location of the motor
     * @param unit unit of speed
     */
    double getMotorSpeed(MotorLocation location, SPEED_UNIT unit);

    /**
     * sets chassis speeds limits
     *
     * @param max_Vel maximum linear velocity of chassis
     * @param max_vOmega maximum angular velocity of chassis
     */
    void setOmniKinematicsLimits(double max_Vel, double max_vOmega);

    /**
     * Yaw motor is a motor that controls the Turret
     * yawOdom is the world pose representation of our robot
     * (basically which direction you want your Heading to be w.r.t Yaw Motor)
     */
    bool setOdomReference();

    
    private:
    DJIMotor LF, RF, LB, RB;
    DJIMotor *yaw = 0;
    MA4 *encoder = nullptr;
    double yawPhase;
    
    // double beybladeSpeed;
    IMU::EulerAngles imuAngles;
    
    ChassisSpeeds desiredChassisSpeeds;
    WheelSpeeds desiredWheelSpeeds;

    OmniKinematicsLimits m_OmniKinematicsLimits;
    WheelSpeeds chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds);
    ChassisSpeeds wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds);

    ChassisSpeeds m_chassisSpeeds;
    WheelSpeeds m_wheelSpeeds;

    uint32_t lastPIDTime = 0;
    unsigned long last_torque_time = 0;
    float curr_fit(int x);

    double yawOdom;
    double imuOdom;
    int testData[300][4];
    int testDataIndex = 0;

    static double radiansToTicks(double radians);
    static double ticksToRadians(double ticks);

    void updateYawPhaseFromEncoder();
    
    static double rpmToRadPerSecond(double RPM);
    static double radPerSecondToRPM(double radPerSecond);

    OmniKinematics m_OmniKinematics;
    float chassis_radius;
    void setOmniKinematics(double radius, HOLONOMIC_MODE mode = OMNI);

    double FF_Ks;
};

#endif // TR_EMBEDDED_CHASSIS_SUBSYSTEM_H