#include "OmniWheelSubsystem.h"
#include <cmath>
#include <cassert>
#include <algorithm>
#include <us_ticker_defines.h>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

OmniWheelSubsystem::OmniWheelSubsystem(const Config &cfg, MA4 *encoder)
    : power_limit(cfg.power_limit_watts),
      LF(DJIMotor::config({ short(cfg.lf_can_id), CANHandler::CANBUS_1, M3508, "left_front",  cfg.lf_pid })),
      RF(DJIMotor::config({ short(cfg.rf_can_id), CANHandler::CANBUS_1, M3508, "right_front", cfg.rf_pid })),
      LB(DJIMotor::config({ short(cfg.lb_can_id), CANHandler::CANBUS_1, M3508, "left_back",   cfg.lb_pid })),
      RB(DJIMotor::config({ short(cfg.rb_can_id), CANHandler::CANBUS_1, M3508, "right_back",  cfg.rb_pid })),
      m_encoder(encoder),
      m_yawOffsetDeg(cfg.yaw_initial_offset_deg),
      m_maxWheelSpeedMps(DEFAULT_MAX_WHEEL_MPS),
      m_maxOmegaRadps(DEFAULT_MAX_OMEGA_RADPS),
      m_beybladeMaxOmega(cfg.max_beyblade_omega_radps),
      m_beybladeRefPowerW(cfg.power_limit_watts)
{
    LF.outputCap = RF.outputCap = LB.outputCap = RB.outputCap = 16000;
    initKinematics(cfg.chassis_radius, cfg.chassis_type);
}

// ─────────────────────────────────────────────────────────────────────────────
// Kinematics setup
// ─────────────────────────────────────────────────────────────────────────────

void OmniWheelSubsystem::initKinematics(double chassisRadius, HolonomicMode mode)
{
    if (mode == OMNI) {
        // Each wheel sits at 45° to the robot axes.
        // Its moment-arm components in X and Y are both (radius / √2).
        // The vOmega term uses (r_x + r_y) = radius * √2.
        m_kinL = chassisRadius * std::sqrt(2.0);
    } else { // MECANUM
        // Axis-aligned wheels; moment arm = half-width + half-length.
        m_kinL = MECANUM_HALF_X + MECANUM_HALF_Y;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Periodic update  (call every control tick)
// ─────────────────────────────────────────────────────────────────────────────

void OmniWheelSubsystem::periodic(const IMU::EulerAngles &imu)
{
    m_imu = imu;

    // Read wheel tangential speeds [m/s] from motor encoders
    m_wheelSpeeds = {
        getWheelSpeedMps(LEFT_FRONT),
        getWheelSpeedMps(RIGHT_FRONT),
        getWheelSpeedMps(LEFT_BACK),
        getWheelSpeedMps(RIGHT_BACK),
    };

    // Derive robot-frame chassis speeds via inverse kinematics
    m_chassisSpeeds = wheelToChassis(m_wheelSpeeds);
}

// ─────────────────────────────────────────────────────────────────────────────
// High-level drive  (public)
// ─────────────────────────────────────────────────────────────────────────────

float OmniWheelSubsystem::setChassisSpeeds(ChassisSpeeds desired, DriveMode mode)
{
    // ── Step 1: Resolve coordinate frame → robot frame ────────────────────────
    ChassisSpeeds robotFrame;

    switch (mode) {

        case ROBOT_ORIENTED:
            robotFrame = desired;
            break;

        case YAW_ORIENTED: {
            // Field-frame input; rotate into robot frame using current encoder heading.
            double headingDeg = getEncoderYawDeg();
            robotFrame = rotateToRobotFrame(desired, headingDeg);
            break;
        }

        case ODOM_ORIENTED: {
            // Fuse encoder and IMU to estimate heading relative to the stored reference.
            // The IMU captures slow drift that the encoder misses (slip, etc.) and vice versa.
            double encoderNow    = getEncoderYawDeg();
            double imuDeltaDeg   = m_odomImuRefDeg     - m_imu.yaw;  // IMU drift since ref
            double encDeltaDeg   = m_odomEncoderRefDeg - encoderNow; // encoder change since ref
            double fusedDeg      = m_odomEncoderRefDeg + (imuDeltaDeg - encDeltaDeg);
            while (fusedDeg >  360.0) fusedDeg -= 360.0;
            while (fusedDeg <    0.0) fusedDeg += 360.0;
            robotFrame = rotateToRobotFrame(desired, fusedDeg);
            break;
        }

        case BEYBLADE: {
            // ── Lateral component (field-frame → robot frame) ──────────────────
            // The chassis is spinning, so we must field-orient the lateral input
            // every tick so the driver always pushes in the direction they intend.
            double headingDeg = getEncoderYawDeg();
            ChassisSpeeds lateral = rotateToRobotFrame({desired.vX, desired.vY, 0.0}, headingDeg);

            // ── Beyblade omega budget ──────────────────────────────────────────
            //
            // Power model (exact for omni kinematics, no cross terms):
            //   Σ(v_wheel²) = 4·(vX² + vY² + (ω·L)²)
            //
            // Scale ω_max to the current power_limit (P ∝ ω² so ω ∝ √P):
            //   ω_max_actual = ω_max_ref · √(power_limit / power_limit_ref)
            //
            // Solve for ω that fills the remaining budget after lateral:
            //   ω_available = √(max(0, ω_max_actual² − (vX² + vY²) / L²))
            //
            double lateralSpeedSq = lateral.vX * lateral.vX + lateral.vY * lateral.vY;
            double omegaSq        = m_beybladeMaxOmega * m_beybladeMaxOmega
                                    - lateralSpeedSq / (m_kinL * m_kinL);
            double omegaAvail     = (omegaSq > 0.0) ? std::sqrt(omegaSq) : 0.0;

            // Positive omega = CCW spin.  Negate here for CW if preferred.
            robotFrame = { lateral.vX, lateral.vY, omegaAvail };
            break;
        }
    }

    // ── Step 2: Forward kinematics → per-wheel tangential speeds [m/s] ────────
    WheelSpeeds wheelMps = chassisToWheel(robotFrame);

    // ── Step 3: Clamp — no wheel may exceed m_maxWheelSpeedMps ────────────────
    wheelMps = normalizeWheelSpeeds(wheelMps);

    // ── Step 4: PID + power budget → motor set-points ─────────────────────────
    return setWheelSpeeds(wheelMps);
}

void OmniWheelSubsystem::setOdomReference()
{
    m_odomEncoderRefDeg = getEncoderYawDeg();
    m_odomImuRefDeg     = m_imu.yaw;
}

// ─────────────────────────────────────────────────────────────────────────────
// Kinematics
// ─────────────────────────────────────────────────────────────────────────────

/*
 * Forward kinematics — robot frame → individual wheel tangential speeds
 *
 * Wheel layout (top view, +X = robot forward):
 *
 *         +X
 *          ↑
 *    LF ───┼─── RF      LF/RB wheels roll along [+1, −1]/√2
 *          │             RF/LB wheels roll along [−1, −1]/√2
 *    LB ───┼─── RB  →+Y
 *
 * This gives the Jacobian rows:
 *   v_LF = +vX  −  vY  −  vOmega · L
 *   v_RF = −vX  −  vY  −  vOmega · L
 *   v_LB = +vX  +  vY  −  vOmega · L
 *   v_RB = −vX  +  vY  −  vOmega · L
 *
 * where L = m_kinL (the effective rotation moment arm [m]).
 * All values in m/s.
 */
WheelSpeeds OmniWheelSubsystem::chassisToWheel(ChassisSpeeds cs) const
{
    const double L = m_kinL;
    return {
        +cs.vX - cs.vY - cs.vOmega * L,   // LF
        -cs.vX - cs.vY - cs.vOmega * L,   // RF
        +cs.vX + cs.vY - cs.vOmega * L,   // LB
        -cs.vX + cs.vY - cs.vOmega * L,   // RB
    };
}

/*
 * Inverse kinematics — wheel tangential speeds → robot frame
 *
 * Derived by left-multiplying the pseudoinverse of the 4×3 Jacobian above.
 * You can verify these by substituting the forward-kinematics rows:
 *
 *   vX     =  ( LF − RF + LB − RB) / 4
 *   vY     =  (−LF − RF + LB + RB) / 4
 *   vOmega = −(LF + RF + LB + RB) / (4 · L)
 *
 * NOTE: the original code had vX = (LF+RF−LB−RB)/4 and vY = (LF−RF+LB−RB)/4,
 * which is wrong — those expressions evaluate to −vY and +vX respectively.
 */
ChassisSpeeds OmniWheelSubsystem::wheelToChassis(WheelSpeeds ws) const
{
    const double L = m_kinL;
    return {
        ( ws.LF - ws.RF + ws.LB - ws.RB) / 4.0,          // vX
        (-ws.LF - ws.RF + ws.LB + ws.RB) / 4.0,          // vY
        -(ws.LF + ws.RF + ws.LB + ws.RB) / (4.0 * L),    // vOmega
    };
}

WheelSpeeds OmniWheelSubsystem::normalizeWheelSpeeds(WheelSpeeds ws) const
{
    double maxAbs = std::max({ std::abs(ws.LF), std::abs(ws.RF),
                               std::abs(ws.LB), std::abs(ws.RB) });

    if (maxAbs > m_maxWheelSpeedMps) {
        ws *= (m_maxWheelSpeedMps / maxAbs);
    }
    return ws;
}

// ─────────────────────────────────────────────────────────────────────────────
// Field-orientation helpers
// ─────────────────────────────────────────────────────────────────────────────

float OmniWheelSubsystem::getEncoderYawDeg() const
{
    // Invert the encoder direction to match the CCW-positive convention.
    float deg = m_encoder->encoderMovingAverage();
    while (deg <   0.0f) deg += 360.0f;
    while (deg > 360.0f) deg -= 360.0f;
    return deg;
}

/*
 * Rotate field-frame ChassisSpeeds into robot frame.
 *
 * theta = (headingDeg − m_yawOffsetDeg) is the CCW angle from the encoder's
 * "field +X" zero to the robot's current heading.
 *
 * vOmega is frame-independent and passes through unchanged.
 */
ChassisSpeeds OmniWheelSubsystem::rotateToRobotFrame(ChassisSpeeds fieldSpeeds,
                                                       double headingDeg) const
{
    double theta = (headingDeg - m_yawOffsetDeg) * OMNI_PI / 180.0;
    double c = std::cos(theta), s = std::sin(theta);
    return {
        fieldSpeeds.vX * c - fieldSpeeds.vY * s,
        fieldSpeeds.vX * s + fieldSpeeds.vY * c,
        fieldSpeeds.vOmega,
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// Unit conversions
// ─────────────────────────────────────────────────────────────────────────────

/*
 * getData(VELOCITY) returns the wheel OUTPUT-SHAFT angular velocity in rad/s.
 * The DJIMotor driver already divides the raw encoder velocity by the gear
 * ratio — do NOT apply M3508_GEAR_RATIO again here.
 *
 * Derived quantities:
 *   wheel_mps  = omega_wheel [rad/s]  × WHEEL_RADIUS_M
 *   motor_rpm  = omega_wheel [rad/s]  × M3508_GEAR_RATIO  × (60 / 2π)
 */

double OmniWheelSubsystem::getWheelSpeedMps(MotorLocation loc)
{
    double omegaWheelRadps = getMotor(loc).getData(VELOCITY); // [rad/s], post gear reduction
    return omegaWheelRadps * WHEEL_RADIUS_M;                  // → [m/s]
}

double OmniWheelSubsystem::getMotorShaftRpm(MotorLocation loc)
{
    double omegaWheelRadps = getMotor(loc).getData(VELOCITY); // [rad/s]
    return omegaWheelRadps * M3508_GEAR_RATIO * (60.0 / (2.0 * OMNI_PI)); // → [RPM]
}

double OmniWheelSubsystem::wheelMpsToMotorRpm(double mps)
{
    // mps → omega_wheel rad/s → motor shaft RPM
    return (mps / WHEEL_RADIUS_M) * M3508_GEAR_RATIO * (60.0 / (2.0 * OMNI_PI));
}

// ─────────────────────────────────────────────────────────────────────────────
// Low-level motor drive
// ─────────────────────────────────────────────────────────────────────────────

float OmniWheelSubsystem::setWheelSpeeds(WheelSpeeds targetMps)
{
    // Convert desired wheel tangential speeds [m/s] → motor shaft RPM (pre-gearbox)
    float targetMotorRpm[4] = {
        (float)wheelMpsToMotorRpm(targetMps.LF),
        (float)wheelMpsToMotorRpm(targetMps.RF),
        (float)wheelMpsToMotorRpm(targetMps.LB),
        (float)wheelMpsToMotorRpm(targetMps.RB),
    };

    // Rate-limit each motor to avoid large current spikes.
    // m_prevMotorRpm holds the actual motor shaft RPM read at the end of the last tick.
    float cmdRpm[4] = {
        limitAcceleration(targetMotorRpm[0], m_prevMotorRpm[0], LF.getData(POWEROUT)),
        limitAcceleration(targetMotorRpm[1], m_prevMotorRpm[1], RF.getData(POWEROUT)),
        limitAcceleration(targetMotorRpm[2], m_prevMotorRpm[2], LB.getData(POWEROUT)),
        limitAcceleration(targetMotorRpm[3], m_prevMotorRpm[3], RB.getData(POWEROUT)),
    };

    uint32_t now = us_ticker_read();
    uint32_t dt  = now - m_lastPidUs;

    // Run speed PID.
    // setpoint  = rate-limited motor shaft RPM command
    // feedback  = actual motor shaft RPM from previous tick (m_prevMotorRpm)
    // The DJIMotor speed-PID output must be scaled by M3508_GEAR_RATIO to
    // produce the correct raw motor power magnitude.
    int power[4] = {
        (int)(M3508_GEAR_RATIO * LF.calculateSpeedPID(cmdRpm[0], m_prevMotorRpm[0], dt)),
        (int)(M3508_GEAR_RATIO * RF.calculateSpeedPID(cmdRpm[1], m_prevMotorRpm[1], dt)),
        (int)(M3508_GEAR_RATIO * LB.calculateSpeedPID(cmdRpm[2], m_prevMotorRpm[2], dt)),
        (int)(M3508_GEAR_RATIO * RB.calculateSpeedPID(cmdRpm[3], m_prevMotorRpm[3], dt)),
    };

    if (targetMotorRpm[0] == 0 && targetMotorRpm[1] == 0 && targetMotorRpm[2] == 0 && targetMotorRpm[3] == 0) {
        power[0] = 0;
        power[1] = 0;
        power[2] = 0;
        power[3] = 0;
    }
    m_lastPidUs = now;

    // Snapshot actual motor shaft RPM for the next tick's rate-limit and PID feedback.
    m_prevMotorRpm[0] = (float)getMotorShaftRpm(LEFT_FRONT);
    m_prevMotorRpm[1] = (float)getMotorShaftRpm(RIGHT_FRONT);
    m_prevMotorRpm[2] = (float)getMotorShaftRpm(LEFT_BACK);
    m_prevMotorRpm[3] = (float)getMotorShaftRpm(RIGHT_BACK);

    // Power budget: estimate total chassis draw and scale down if over budget.
    float totalEstimatedWatts = estimatePowerWatts(LF.getData(TORQUE))
                               + estimatePowerWatts(RF.getData(TORQUE))
                               + estimatePowerWatts(LB.getData(TORQUE))
                               + estimatePowerWatts(RB.getData(TORQUE));

    // A small epsilon in the denominator prevents division by zero when idle.
    constexpr float POWER_MARGIN_W = 10.0f;
    float scale = std::min(1.0f, power_limit / (totalEstimatedWatts + POWER_MARGIN_W));
    // printf("s: %.2f, %d %d %d %d\n", totalEstimatedWatts, power[0], power[1], power[2], power[3]);
    // printf("%.2f\n", power_limit);

    LF.setPower(power[0] * scale);
    RF.setPower(power[1] * scale);
    LB.setPower(power[2] * scale);
    RB.setPower(power[3] * scale);

    return scale;
}

/*
 * Rate-limit the commanded motor shaft RPM.
 *
 * If the motor is currently unpowered (currentPower == 0) the limit is
 * skipped — there is no risk of overcurrent and the motor needs to spin up.
 *
 * If the direction would flip sign we pass through zero first to avoid a
 * sudden reversal.
 */
float OmniWheelSubsystem::limitAcceleration(float desired, float prev, int currentPower)
{
    constexpr float MAX_DELTA_RPM = 100.0f;

    // Snap to zero before reversing direction
    bool reversing = (desired > 0.0f && prev < 0.0f) || (desired < 0.0f && prev > 0.0f);
    if (reversing) return 0.0f;

    if (desired == 0.0f) return 0.0f;

    // Skip limiting when the motor is unpowered (no overcurrent risk)
    if (currentPower == 0) return desired;

    float delta = desired - prev;
    if      (delta >  MAX_DELTA_RPM) return prev + MAX_DELTA_RPM;
    else if (delta < -MAX_DELTA_RPM) return prev - MAX_DELTA_RPM;
    else                             return desired;
}

/*
 * Estimate instantaneous motor power [W] from the raw torque register.
 *
 * Piecewise-linear current model, calibrated for M3508:
 *   • linear region:    I = (|counts| / PEAK_COUNTS) × LINEAR_SLOPE  [A]
 *   • saturation region (ratio > SATURATION_RATIO): I = SATURATION_CURRENT [A]
 *
 * Assumed bus voltage: 24 V.  Power = 24 V × I_total.
 */
float OmniWheelSubsystem::estimatePowerWatts(int torqueCounts)
{
    constexpr int   PEAK_TORQUE_COUNTS  = 5596;
    constexpr float SATURATION_RATIO    = 0.4375f;
    constexpr float SATURATION_CURRENT  = 1.22f;   // [A]
    constexpr float LINEAR_SLOPE        = 14.0f / 4.9f;
    constexpr float BUS_VOLTAGE         = 24.0f;   // [V]

    float ratio = std::abs(static_cast<float>(torqueCounts)) / PEAK_TORQUE_COUNTS;

    float currentA;
    if (ratio > SATURATION_RATIO) {
        // Log an over-torque event at most once every 200 ms
        if ((us_ticker_read() - m_lastTorqueUs) / 1000UL > 200UL) {
            m_lastTorqueUs = us_ticker_read();
        }
        currentA = SATURATION_CURRENT;
    } else {
        currentA = ratio * LINEAR_SLOPE;
    }

    return BUS_VOLTAGE * currentA;
}

// ─────────────────────────────────────────────────────────────────────────────
// Motor access / configuration
// ─────────────────────────────────────────────────────────────────────────────

DJIMotor &OmniWheelSubsystem::getMotor(MotorLocation loc)
{
    switch (loc) {
        case LEFT_FRONT:  return LF;
        case RIGHT_FRONT: return RF;
        case LEFT_BACK:   return LB;
        case RIGHT_BACK:  return RB;
    }
    assert(false && "invalid MotorLocation");
    __builtin_unreachable();
}

void OmniWheelSubsystem::setMotorSpeedPID(MotorLocation loc, float kP, float kI, float kD)
{
    getMotor(loc).setSpeedPID(kP, kI, kD);
}

void OmniWheelSubsystem::setSpeedLimits(double maxLinearMps, double maxOmegaRadps)
{
    m_maxWheelSpeedMps = maxLinearMps;
    m_maxOmegaRadps    = maxOmegaRadps;
}