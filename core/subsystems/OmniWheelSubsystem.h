#pragma once

#include "mbed.h"
#include "util/peripherals/imu/IMU.h"
#include "util/peripherals/encoder/MA4.h"
#include "util/motor/DJIMotor.h"
#include "util/communications/CANHandler.h"
#include "util/algorithms/PID.h"

#include <cstdint>

// ── Physical / hardware constants ──────────────────────────────────────────────
// Adjust if hardware changes.

static constexpr double OMNI_PI              = 3.14159265358979;
static constexpr double WHEEL_RADIUS_M       = 0.073;   //< 146 mm diameter / 2  [m]
static constexpr double MECANUM_HALF_X       = 0.14;    //< wheel-centre to chassis X-axis [m]
static constexpr double MECANUM_HALF_Y       = 0.21;    //< wheel-centre to chassis Y-axis [m]
static constexpr double DEFAULT_MAX_WHEEL_MPS  = 5;  //< linear wheel speed cap  [m/s]
static constexpr double DEFAULT_MAX_OMEGA_RADPS = 8.0;  //< angular velocity cap  [rad/s]
static constexpr float STATIC_FRICTION_CONSTANT = 0.233924f;
static constexpr float STATIC_FRICTION_CONSTANT_ALT = 0.8f;
static constexpr float GRAVITY = 9.80665f;
static constexpr int ACCEL_DENOM_CONSTANT = 2;
static constexpr int MIN_MOTOR_RPM = 1000;

// ── Kinematic types ────────────────────────────────────────────────────────────

// Tangential speed at each wheel hub [m/s].
// Sign convention: positive means the wheel rolls in the "forward-contributing" direction.
struct WheelSpeeds {
    double LF, RF, LB, RB;
    void operator*=(double s) { LF *= s; RF *= s; LB *= s; RB *= s; }
};

// Robot-frame chassis velocity.
// +X = forward, +Y = left, +Ω = counter-clockwise (right-hand Z-up).
struct ChassisSpeeds {
    double vX;      //< forward velocity    [m/s]
    double vY;      //< leftward velocity   [m/s]
    double vOmega;  //< CCW angular rate    [rad/s]
};

// ──────────────────────────────────────────────────────────────────────────────

/**
 * OmniWheelSubsystem — velocity-controlled 4-wheel holonomic drive.
 *
 * Coordinate frame (robot frame):
 *   +X  forward   +Y  left   +Ω  counter-clockwise (right-hand, Z-up)
 *
 * Wheel layout (top view, robot nose points up):
 *
 *        +X (forward)
 *         ↑
 *   LF ───┼─── RF
 *         │
 *   LB ───┼─── RB      → +Y (left)
 *
 * In OMNI mode each wheel is mounted at 45° to the robot X-axis.
 * In MECANUM mode wheels are axis-aligned with 45° rollers.
 */
class OmniWheelSubsystem {
public:

    enum MotorLocation { LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK };

    enum DriveMode {
        ROBOT_ORIENTED,  //< vX/vY in robot frame  (+X = robot nose)
        YAW_ORIENTED,    //< vX/vY in field frame, heading sourced from turret encoder
        ODOM_ORIENTED,   //< vX/vY in field frame, heading from fused encoder + IMU
        BEYBLADE,
    };

    enum HolonomicMode { OMNI, MECANUM };

    struct Config {
        int lf_can_id, rf_can_id, lb_can_id, rb_can_id; //< motor CAN IDs
        PID::config lf_pid, rf_pid, lb_pid, rb_pid;     //< per-motor speed PID configs

        double chassis_radius;         //< wheel-centre to chassis-centre distance [m]

        // Encoder reading (degrees) that corresponds to the robot facing field +X.
        // Used to zero out field-oriented drive.
        float yaw_initial_offset_deg;

        float power_limit_watts = 60.f; //< total chassis power budget [W]

        // Maximum beyblade spin rate at power_limit_watts with no lateral movement [rad/s].
        double max_beyblade_omega_radps = 8.0;

        HolonomicMode chassis_type = OMNI;
    };

    explicit OmniWheelSubsystem(const Config &cfg, MA4 *encoder);

    // ── Main loop ──────────────────────────────────────────────────────────────

    // Must be called every control-loop iteration.
    // Updates wheel-speed and chassis-speed odometry from motor encoders + IMU.
    void periodic(const IMU::EulerAngles &imu);

    // ── Drive commands ─────────────────────────────────────────────────────────

    // Set chassis velocity.
    // @param speeds  desired [vX m/s, vY m/s, vOmega rad/s] in the chosen frame
    // @param mode    coordinate frame for vX / vY (vOmega is always robot-relative)
    // @return        power-budget scale factor in [0, 1]  (1.0 = no limiting)
    float setChassisSpeeds(ChassisSpeeds speeds, unsigned long dt_s, DriveMode mode = ROBOT_ORIENTED);

    // Snapshot the current heading as the reference origin for ODOM_ORIENTED mode.
    void setOdomReference();

    // ── Odometry ──────────────────────────────────────────────────────────────

    WheelSpeeds   getWheelSpeeds()   const { return m_wheelSpeeds;   }
    ChassisSpeeds getChassisSpeeds() const { return m_chassisSpeeds; }

    // ── Runtime configuration ──────────────────────────────────────────────────

    // Override the default speed limits.
    void setSpeedLimits(double maxLinearMps, double maxOmegaRadps);

    // ── Motor-level access (for PID tuning / telemetry) ────────────────────────

    DJIMotor &getMotor(MotorLocation loc);
    void setMotorSpeedPID(MotorLocation loc, float kP, float kI, float kD);

    float power_limit; //< chassis power budget [W] — may be changed at runtime

private:
    // ── Hardware ───────────────────────────────────────────────────────────────
    DJIMotor LF, RF, LB, RB;
    MA4     *m_encoder;

    // ── Odometry state ─────────────────────────────────────────────────────────
    WheelSpeeds      m_wheelSpeeds   = {};
    ChassisSpeeds    m_chassisSpeeds = {};
    IMU::EulerAngles m_imu           = {};

    // ── Field-orientation ──────────────────────────────────────────────────────
    float  m_yawOffsetDeg;          //< encoder value == "robot facing field +X"  [deg]

    // Snapshots stored by setOdomReference() for ODOM_ORIENTED heading fusion.
    double m_odomEncoderRefDeg = 0.0;
    double m_odomImuRefDeg     = 0.0;

    // ── Kinematics ─────────────────────────────────────────────────────────────
    // Effective moment arm for the vOmega term: (r_x + r_y) [m].
    // For OMNI:    chassis_radius * √2
    // For MECANUM: MECANUM_HALF_X + MECANUM_HALF_Y
    double m_kinL;

    double m_maxWheelSpeedMps;
    double m_maxOmegaRadps;

    // ── Beyblade ───────────────────────────────────────────────────────────────
    // Calibrated ω_max [rad/s] at the reference power level (= power_limit_watts
    // from the constructor config).  The actual ω_max is derived at runtime by
    // scaling this value with √(power_limit / power_limit).
    double m_beybladeMaxOmega;     //< ω_max at reference power  [rad/s]

    // ── PID / rate-limiting state ──────────────────────────────────────────────
    uint32_t      m_lastPidUs    = 0;
    unsigned long m_lastTorqueUs = 0;

    // Motor-shaft RPM (pre-gearbox) measured at the END of the previous tick.
    // Used as both the rate-limit reference and the PID feedback.
    float m_prevMotorRpm[4] = {0, 0, 0, 0};

    // ── Internal helpers ───────────────────────────────────────────────────────

    void initKinematics(double chassisRadius, HolonomicMode mode);

    // ─ Unit conversions ─────────────────────────────────────────────────────
    //
    // DJIMotor::getData(VELOCITY) returns the wheel output-shaft angular velocity
    // in rad/s (i.e. the raw encoder velocity has already been divided by the
    // gear ratio inside the driver).  Everything below is derived from that.

    // Wheel tangential speed from encoder [m/s].
    //   wheel_mps = omega_wheel_radps * WHEEL_RADIUS_M
    double getWheelSpeedMps(MotorLocation loc);

    // Motor shaft speed (pre-gearbox) from encoder [RPM].
    //   motor_rpm = omega_wheel_radps * M3508_GEAR_RATIO * (60 / 2π)
    double getMotorShaftRpm(MotorLocation loc);

    // Convert wheel tangential speed to motor shaft RPM.
    //   motor_rpm = (mps / WHEEL_RADIUS_M) * M3508_GEAR_RATIO * (60 / 2π)
    static double wheelMpsToMotorRpm(double mps);

    // ─ Kinematics ───────────────────────────────────────────────────────────

    // Forward kinematics: robot-frame speeds [m/s, m/s, rad/s] → wheel tangential speeds [m/s].
    WheelSpeeds chassisToWheel(ChassisSpeeds cs) const;

    // Inverse kinematics: wheel tangential speeds [m/s] → robot-frame speeds [m/s, m/s, rad/s].
    // This is the exact pseudoinverse of chassisToWheel.
    ChassisSpeeds wheelToChassis(WheelSpeeds ws) const;

    // Scale wheel speeds down proportionally so no wheel exceeds m_maxWheelSpeedMps.
    WheelSpeeds normalizeWheelSpeeds(WheelSpeeds ws) const;

    // ─ Field-orientation ────────────────────────────────────────────────────

    // Read encoder heading, normalised to [0, 360) degrees.
    float getEncoderYawDeg() const;

    // Rotate field-frame desired speeds into robot frame.
    // @param headingDeg  current robot heading in field frame, CCW-positive [deg]
    ChassisSpeeds rotateToRobotFrame(ChassisSpeeds fieldSpeeds, double headingDeg, unsigned long dt_s) const;

    // Calculate max available beyblad velocity
    double CalculateBeybladeVelo(float vOmega, ChassisSpeeds lateral);

    double calibrateMaxBeybladeOmega();

    // ─ Motor drive ──────────────────────────────────────────────────────────

    // Drive all four wheels at the given tangential speeds [m/s].
    // Converts to motor shaft RPM, applies rate-limiting, runs PID, and
    // enforces the power budget.
    // @return power-limit scale factor [0, 1]
    float setWheelSpeeds(WheelSpeeds targetMps);

    // Clamp the per-tick RPM change to prevent large current spikes.
    static float limitAcceleration(float desiredRPM, float previousRPM, uint32_t deltaTime, float theta);

    // Estimate instantaneous motor power [W] from the raw torque register.
    float estimatePowerWatts(int torqueCounts);
};