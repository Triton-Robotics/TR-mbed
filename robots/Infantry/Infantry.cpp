#include "base_robot/BaseRobot.h"
#include "util/algorithms/general_functions.h"

#include "subsystems/ChassisSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"

#include "util/communications/CANHandler.h"
#include "util/communications/jetson/Jetson.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"

#include <algorithm>
#include <cmath>

constexpr auto IMU_I2C_SDA = PB_7;
constexpr auto IMU_I2C_SCL = PB_8;
constexpr auto IMU_RESET = PA_8;

constexpr int pitch_zero_offset_ticks = 1500;
constexpr float PITCH_LOWER_BOUND{-32.0};
constexpr float PITCH_UPPER_BOUND{35.0};

constexpr float JOYSTICK_YAW_SENSITIVITY_DPS = 600;
constexpr float JOYSTICK_PITCH_SENSITIVITY_DPS = 300;
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

constexpr PID::config YAW_VEL_PID = {350, 0.5, 2.5, 32000, 2000};
constexpr PID::config YAW_POS_PID = {0.5, 0, 0, 90, 2};
constexpr PID::config PITCH_VEL_PID = {500, 0.8, 0, 32000, 2000};
constexpr PID::config PITCH_POS_PID = {1.5, 0.0005, 0.05, 30, 2};

constexpr PID::config FL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config FR_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BR_VEL_CONFIG = {3, 0, 0};

constexpr PID::config FLYWHEEL_L_PID = {7.1849, 0.000042634, 0};
constexpr PID::config FLYWHEEL_R_PID = {7.1849, 0.000042634, 0};
constexpr PID::config INDEXER_PID_VEL = {2.7, 0.001, 0};
constexpr PID::config INDEXER_PID_POS = {0.1, 0, 0.001};

// State variables
ChassisSpeeds des_chassis_state;
TurretSubsystem::TurretInfo des_turret_state;
ShootState des_shoot_state;

int remoteTimer = 0;

float pitch_desired_angle = 0.0;
float yaw_desired_angle = 0.0;

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

class Infantry : public BaseRobot {
  public:
    I2C i2c_;
    BNO055 imu_;

    BufferedSerial jetson_raw_serial;
    Jetson jetson;

    Jetson::WriteState stm_state;
    Jetson::ReadState jetson_state;

    TurretSubsystem turret;
    ShooterSubsystem shooter;
    ChassisSubsystem chassis;

    bool imu_initialized{false};

    Infantry(Config &config)
        : BaseRobot(config),
          // clang-format off
        i2c_(IMU_I2C_SDA, IMU_I2C_SCL),
        imu_(i2c_, IMU_RESET, MODE_IMU),
        jetson_raw_serial(PC_12, PD_2, 115200),
        jetson(jetson_raw_serial),

        turret(TurretSubsystem::config{
            4,
            GM6020,
            7,
            GM6020,
            pitch_zero_offset_ticks,
            YAW_VEL_PID,
            YAW_POS_PID,
            PITCH_VEL_PID,
            PITCH_POS_PID,
            1221.0f, 
            97.4f,
            -2600.0f, 
            1221.0f,
            97.4f,
            CANHandler::CANBUS_1,
            CANHandler::CANBUS_2,
            -1,
            (1.0f/3.0f),
            PITCH_LOWER_BOUND,
            PITCH_UPPER_BOUND
        }, imu_),

        shooter(ShooterSubsystem::config{
            ShooterSubsystem::BURST,
            0,
            1,
            2,
            7,
            FLYWHEEL_L_PID,
            FLYWHEEL_R_PID,
            INDEXER_PID_VEL,
            INDEXER_PID_POS,
            CANHandler::CANBUS_2,
            true
        }),

        chassis(ChassisSubsystem::Config{
            1,        // left_front_can_id
            2,        // right_front_can_id
            3,        // left_back_can_id
            4,        // right_back_can_id
            0.22617,  // radius
            0.065,    // speed_pid_ff_ks
            &turret.yaw,
            356 + 6144,  // yaw_initial_offset_ticks
            imu_
        })
    // clang-format on
    {}

    ~Infantry() {}

    void init() override {}

    void periodic(unsigned long dt_us) override {
        IMU::EulerAngles imuAngles = imu_.read();

        if (!imu_initialized) {
            IMU::EulerAngles angles = imu_.getImuAngles();
            if (angles.pitch == 0.0 && angles.yaw == 0.0 && angles.roll == 0.0) {
                return;
            }
            yaw_desired_angle = angles.yaw;
            imu_initialized = true;
        }

        max_linear_vel = 1.24 + 0.0513 * chassis.power_limit +
                         0.000216 * (chassis.power_limit * chassis.power_limit);
        des_chassis_state.vX = jy * max_linear_vel;
        des_chassis_state.vY = -jx * max_linear_vel;

        // Turret yaw from mouse + right joystick horizontal
        yaw_desired_angle -= myaw * MOUSE_SENSITIVITY_YAW_DPS * dt_us / 1000000;
        yaw_desired_angle -= jyaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_us / 1000000;
        yaw_desired_angle = capAngle(yaw_desired_angle);
        des_turret_state.yaw_angle_degs = yaw_desired_angle;

        // Pitch from mouse + right joystick vertical
        pitch_desired_angle -= mpitch * MOUSE_SENSITIVITY_PITCH_DPS * dt_us / 1000000;
        pitch_desired_angle += jpitch * JOYSTICK_PITCH_SENSITIVITY_DPS * dt_us / 1000000;
        pitch_desired_angle = std::clamp(pitch_desired_angle, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);
        des_turret_state.pitch_angle_degs = pitch_desired_angle;

        // vW: right joystick vertical (jpitch = remote_.rightY()), scaled to -6.1..6.1 rad/s
        omega_speed = jpitch * 4.8f;

        // Read jetson
        jetson_state = jetson.read();

        if ((remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID) ||
             remote_.getMouseR()) {
            des_turret_state.pitch_angle_degs = jetson_state.desired_pitch_rads * (180.0 / M_PI);
            des_turret_state.yaw_angle_degs   = jetson_state.desired_yaw_rads   * (180.0 / M_PI);
        }

        // Chassis logic — ROBOT_ORIENTED so yaw offset doesn't interfere
        if (drive == 'u' || (drive == 'o' && remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP)) {
            des_chassis_state.vOmega = 0;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);
            des_turret_state.turret_mode = AIM;
        } else if (drive == 'd' || (drive == 'o' && remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN)) {
            des_chassis_state.vOmega = omega_speed;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);
            des_turret_state.turret_mode = AIM;
        } else {
            des_chassis_state.vOmega = 0;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);
            des_turret_state.turret_mode = SLEEP;
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

        turret.setState(des_turret_state);
        shooter.setState(des_shoot_state);

        turret.periodic(chassis.getChassisSpeeds().vOmega * 60 / (2 * PI));
        chassis.periodic(&imuAngles);
        shooter.periodic(referee_.power_heat_data.shooter_17mm_1_barrel_heat,
                         referee_.robot_status.shooter_barrel_heat_limit);

        set_jetson_state();
        jetson.write(stm_state);
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop

    void set_jetson_state() {
        stm_state.game_state = 4;
        stm_state.robot_hp = 200;

        // Use actual chassis speeds from motor feedback (m_chassisSpeeds via getter)
        ChassisSpeeds actual = chassis.getChassisSpeeds();
        stm_state.chassis_x_velocity = actual.vX;
        stm_state.chassis_y_velocity = actual.vY;
        stm_state.chassis_rotation   = actual.vOmega;

        stm_state.yaw_angle_rads   = degreesToRadians(turret.getState().yaw_angle_degs);
        stm_state.yaw_velocity     = turret.getState().yaw_velo_rad_s;
        stm_state.pitch_angle_rads = degreesToRadians(turret.getState().pitch_angle_degs);
        stm_state.pitch_velocity   = turret.getState().pitch_velo_rad_s;
    }
};

int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{};
    Infantry infantry(config);

    infantry.main_loop();
}