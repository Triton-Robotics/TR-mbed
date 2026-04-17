#include "base_robot/BaseRobot.h"
#include "util/algorithms/general_functions.h"
#include "ResetReason.h"

#include "subsystems/ChassisSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"

#include "util/communications/CANHandler.h"
#include "util/communications/PwmIn.h"
#include "util/communications/jetson/Jetson.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"
#include "util/peripherals/encoder/MA4.h"

#include <algorithm>
#include <us_ticker_api.h>

constexpr auto IMU_I2C_SDA = PB_7;
constexpr auto IMU_I2C_SCL = PB_8;
constexpr auto IMU_RESET = PA_8;

constexpr int pitch_zero_offset_ticks = 1500;
constexpr float PITCH_LOWER_BOUND{-22.0};
constexpr float PITCH_UPPER_BOUND{20.0};

constexpr float JOYSTICK_YAW_SENSITIVITY_DPS = 600;
constexpr float JOYSTICK_PITCH_SENSITIVITY_DPS = 300;
// Mouse sensitivity initialized
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

constexpr PID::config YAW_VEL_PID     = {181, 3.655 * 10e-3, 4.51 * 2.25, 32000, 1000};
constexpr PID::config YAW_POS_PID     = {1, 0, 0, 90, 2};
const float yaw_static_friction       = -150;
const float yaw_kinetic_friction      = 0;

constexpr PID::config PITCH_VEL_PID   = {173.8994, 4.898 * 10e-9, 12.474 * 10, 16000, 1000};
constexpr PID::config PITCH_POS_PID   = {1, 0, 0, 30, 2};
const float pitch_gravity_feedforward = -500;
const float pitch_static_friction     = 635.0 / 5;
const float pitch_kinetic_friction    = 0;

constexpr PID::config FL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config FR_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BR_VEL_CONFIG = {3, 0, 0};

constexpr PID::config FLYWHEEL_L_PID  = {7.1849, 0.000042634, 0};
constexpr PID::config FLYWHEEL_R_PID  = {7.1849, 0.000042634, 0};
constexpr PID::config INDEXER_PID_VEL = {2.7, 0.001, 0};
constexpr PID::config INDEXER_PID_POS = {0.1, 0, 0.001};


// Config variables
TurretSubsystem::config turret_config = {
    7,
    M3508,
    8,
    M3508,
    pitch_zero_offset_ticks,
    YAW_VEL_PID,
    YAW_POS_PID,
    PITCH_VEL_PID,
    PITCH_POS_PID,
    yaw_static_friction,
    yaw_kinetic_friction,
    pitch_gravity_feedforward,
    pitch_static_friction,
    pitch_kinetic_friction,
    CANHandler::CANBUS_1,
    CANHandler::CANBUS_2,
    1,
    (1.0 / 3.0),
    PITCH_LOWER_BOUND,
    PITCH_UPPER_BOUND
};
ShooterSubsystem::config shooter_config = {
    ShooterSubsystem::BURST,
    0,
    1,
    2,
    6,
    FLYWHEEL_L_PID,
    FLYWHEEL_R_PID,
    INDEXER_PID_VEL,
    INDEXER_PID_POS,
    CANHandler::CANBUS_2,
    true
};

// State variables
ChassisSpeeds des_chassis_state;
TurretSubsystem::TurretInfo des_turret_state;
ShootState des_shoot_state;

int remoteTimer = 0;

float pitch_desired_angle = 0.0;
float yaw_desired_angle = 0.0;
float dt_global = 0.0;
unsigned long timer = 0;

IMU::EulerAngles imuAngles;
class Infantry : public BaseRobot {
  public:
    I2C i2c_;
    BNO055 imu_;
    MA4 encoder_;
    BufferedSerial jetson_raw_serial;
    Jetson jetson;

    Jetson::WriteState stm_state;
    Jetson::ReadState jetson_state;

    TurretSubsystem turret_;
    ShooterSubsystem shooter_;
    ChassisSubsystem chassis_;

    bool imu_initialized{false};

    Infantry(Config &config)
        : BaseRobot(config),
          // clang-format off
        i2c_(IMU_I2C_SDA, IMU_I2C_SCL),
        imu_(i2c_, IMU_RESET, MODE_IMU),
        encoder_(PB_4),
        jetson_raw_serial(PC_12, PD_2, 115200),
        jetson(jetson_raw_serial),
        turret_(turret_config, imu_),
        shooter_(shooter_config),
        chassis_(ChassisSubsystem::Config{
            1,        // left_front_can_id
            2,        // right_front_can_id
            3,        // left_back_can_id
            4,        // right_back_can_id
            0.22617,  // radius
            0.065,    // speed_pid_ff_ks
            &turret_.yaw,
            86,       // yaw_initial_offset_ticks
            imu_,
            &encoder_
        })
    // clang-format on
    {
    }

    ~Infantry() {}

    void init() override {
        timer = us_ticker_read();
    }

    void periodic(unsigned long dt_us) override {
        imuAngles = imu_.read();

        if (!imu_initialized) {
            IMU::EulerAngles angles = imu_.getImuAngles();
            if (angles.pitch == 0.0 && angles.yaw == 0.0 && angles.roll == 0.0) {
                return;
            }
            imu_initialized = true;
        }

        max_linear_vel = 1.24 + 0.0513 * chassis_.power_limit +
                         0.000216 * (chassis_.power_limit * chassis_.power_limit);
        des_chassis_state.vX = jy * max_linear_vel;
        des_chassis_state.vY = jx * max_linear_vel;

        // Turret yaw from remote
        yaw_desired_angle -= myaw * MOUSE_SENSITIVITY_YAW_DPS * dt_us / 1000000;
        yaw_desired_angle -= jyaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_us / 1000000;
        yaw_desired_angle = capAngle(yaw_desired_angle);
        des_turret_state.yaw_angle_degs = yaw_desired_angle;

        // Pitch from mouse only (right joystick vertical now controls vW)
        pitch_desired_angle -= mpitch * MOUSE_SENSITIVITY_PITCH_DPS * dt_us / 1000000;
        pitch_desired_angle = std::clamp(pitch_desired_angle, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);
        des_turret_state.pitch_angle_degs = pitch_desired_angle;

        // vW (chassis spin) controlled by right joystick vertical
        // jpitch is right joystick vertical, scaled to -6.1..6.1 rad/s
        omega_speed = jpitch * 3.0f;

        // Read jetson
        jetson_state = jetson.read();

        // Chassis logic
        if (drive == 'u' || (drive == 'o' && remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) ==
                                                 Remote::SwitchState::UP)) {
            des_chassis_state.vOmega = 0;
            chassis_.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state.turret_mode = AIM;
        } else if (drive == 'd' ||
                   (drive == 'o' &&
                    remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN)) {
            des_chassis_state.vOmega = omega_speed;
            chassis_.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state.turret_mode = AIM;
        } else {
            des_chassis_state.vOmega = 0;
            chassis_.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state.turret_mode = SLEEP;
            des_turret_state.yaw_angle_degs = turret_.getState().yaw_angle_degs;
            yaw_desired_angle = turret_.getState().yaw_angle_degs;
            des_turret_state.pitch_angle_degs = 0;
        }

        // Shooter Logic
        if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP ||
            remote_.getMouseL()) {
            des_shoot_state = ShootState::SHOOT;
        } else if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID ||
                   shot == 'd') {
            des_shoot_state = ShootState::FLYWHEEL;
        } else {
            des_shoot_state = ShootState::OFF;
        }

        turret_.setState(des_turret_state);
        shooter_.setState(des_shoot_state);

        turret_.periodic(chassis_.getChassisSpeeds().vOmega * 60 / (2 * PI));
        chassis_.periodic(&imuAngles);
        shooter_.periodic(referee_.power_heat_data.shooter_17mm_1_barrel_heat,
                         referee_.robot_status.shooter_barrel_heat_limit);

        set_jetson_state();
        jetson.write(stm_state);
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop

    void set_jetson_state() {
        stm_state.game_state = 4;
        stm_state.robot_hp = 200;

        stm_state.chassis_x_velocity = chassis_.getChassisSpeeds().vX;
        stm_state.chassis_y_velocity = chassis_.getChassisSpeeds().vY;
        stm_state.chassis_rotation = chassis_.getChassisSpeeds().vOmega;

        stm_state.yaw_angle_rads = degreesToRadians(turret_.getState().yaw_angle_degs);
        stm_state.yaw_velocity = degreesToRadians(turret_.getState().yaw_velo_rad_s);
        stm_state.pitch_angle_rads = degreesToRadians(turret_.getState().pitch_angle_degs);
        stm_state.pitch_velocity = degreesToRadians(turret_.getState().pitch_velo_rad_s);
    }
};

int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{};
    Infantry infantry(config);

    infantry.main_loop();
}