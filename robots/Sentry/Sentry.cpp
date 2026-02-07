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

constexpr auto IMU_I2C_SDA = PB_7;
constexpr auto IMU_I2C_SCL = PB_8;
constexpr auto IMU_RESET = PA_8;

constexpr int pitch_zero_offset_ticks = 1500;
constexpr float PITCH_LOWER_BOUND{-32.0};
constexpr float PITCH_UPPER_BOUND{35.0};

constexpr float JOYSTICK_YAW_SENSITIVITY_DPS = 600;
constexpr float JOYSTICK_PITCH_SENSITIVITY_DPS = 300;

PID::config yaw_vel_PID = {350, 0.5, 2.5, 32000, 2000};
PID::config yaw_pos_PID = {0.5, 0, 0, 90, 2};
PID::config pitch_vel_PID = {500, 0.8, 0, 32000, 2000};
PID::config pitch_pos_PID = {1.5, 0.0005, 0.05, 30, 2};

PID::config fl_vel_config = {3, 0, 0};
PID::config fr_vel_config = {3, 0, 0};
PID::config bl_vel_config = {3, 0, 0};
PID::config br_vel_config = {3, 0, 0};

PID::config flywheelL_PID = {7.1849, 0.000042634, 0};
PID::config flywheelR_PID = {7.1849, 0.000042634, 0};
PID::config indexer_PID_vel = {2.7, 0.001, 0};
PID::config indexer_PID_pos = {0.1, 0, 0.001};

// State variables
ChassisSpeeds des_chassis_state;
TurretSubsystem::TurretInfo des_turret_state;
ShootState des_shoot_state;

int remoteTimer = 0;

float pitch_desired_angle = 0.0;
float yaw_desired_angle = 0.0;


double degreesToRadians(double degrees) {
    // M_PI is a common constant for Pi in cmath
    return degrees * M_PI / 180.0;
}

class Sentry : public BaseRobot {
  public:
    I2C i2c_;
    BNO055 imu_;

    // TODO: put the BufferedSerial inside Jetson (idk if we wanna do that tho
    // for SPI)
    BufferedSerial jetson_raw_serial;
    Jetson jetson;

    Jetson::WriteState stm_state;
    Jetson::ReadState jetson_state;

    TurretSubsystem turret;
    ShooterSubsystem shooter;
    ChassisSubsystem chassis;

    bool imu_initialized{false};

    Sentry(Config &config)
        : BaseRobot(config), i2c_(IMU_I2C_SDA, IMU_I2C_SCL), imu_(i2c_, IMU_RESET, MODE_IMU),
          jetson_raw_serial(PC_12, PD_2,
                            115200), // TODO: check higher baud to see if still works
          jetson(jetson_raw_serial),
          turret(TurretSubsystem::config{4, GM6020, 7, GM6020, pitch_zero_offset_ticks, yaw_vel_PID,
                                         yaw_pos_PID, pitch_vel_PID, pitch_pos_PID,
                                         CANHandler::CANBUS_1, CANHandler::CANBUS_2, -1, imu_,
                                         PITCH_LOWER_BOUND, PITCH_UPPER_BOUND}),

          shooter(ShooterSubsystem::config{ShooterSubsystem::BURST, 0, 1, 2, 7, flywheelL_PID,
                                           flywheelR_PID, indexer_PID_vel, indexer_PID_pos,
                                           CANHandler::CANBUS_2, false}),

          chassis(ChassisSubsystem::Config{1,           // left_front_can_id
                                           2,           // right_front_can_id
                                           3,           // left_back_can_id
                                           4,           // right_back_can_id
                                           0.22617,     // radius
                                           0.065,       // speed_pid_ff_ks
                                           &turret.yaw, // yaw_motor
                                           356,        // yaw_initial_offset_ticks
                                           imu_}) {}

    ~Sentry() {}

    void init() override {}

    void periodic(unsigned long dt_us) override {
        // TODO this should be threaded inside imu instead
        IMU::EulerAngles imuAngles = imu_.read();

        if (!imu_initialized) {
            IMU::EulerAngles angles = imu_.getImuAngles();
            if (angles.pitch == 0.0 && angles.yaw == 0.0 && angles.roll == 0.0) {
                return;
            }

            yaw_desired_angle = angles.yaw;
            imu_initialized = true;
        }

        // Chassis + Turret Logic
        // TODO migrate away from remote chassis/pitch/yaw specific code
        des_chassis_state.vX = remote_.getChannel(Remote::Channel::LEFT_VERTICAL);
        des_chassis_state.vY = -1 * remote_.getChannel(Remote::Channel::LEFT_HORIZONTAL);

        // Turret from remote
        float joystick_yaw = remote_.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
        yaw_desired_angle -= joystick_yaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_us / 1000000;
        yaw_desired_angle = capAngle(yaw_desired_angle);
        des_turret_state.yaw_angle = yaw_desired_angle;

        float joystick_pitch = remote_.getChannel(Remote::Channel::RIGHT_VERTICAL);
        pitch_desired_angle += joystick_pitch * JOYSTICK_PITCH_SENSITIVITY_DPS * dt_us / 1000000;
        pitch_desired_angle = std::clamp(pitch_desired_angle, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);
        des_turret_state.pitch_angle = pitch_desired_angle;

        jetson_state = jetson.read();

        printf("pitch: %.4f | %.4f \nyaw:   %.4f | %.4f\n", 
            jetson_state.desired_pitch_rads * (180.0 / M_PI), turret.getState().pitch_angle,
            jetson_state.desired_yaw_rads * (180 / M_PI), turret.getState().yaw_angle );

        if (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP) {
            des_chassis_state.vOmega = 0;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state.turret_mode = TurretState::AIM;
        } else if (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN) {
            // Jetson odom
            // TODO ------------------ ADD THIS BACK FOR ODOM MOVEMENT --------------------------
            // des_chassis_state.vX = jetson_state.desired_x_vel;
            // des_chassis_state.vY = jetson_state.desired_y_vel;
            // des_chassis_state.vOmega = jetson_state.desired_angular_vel;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);

            des_turret_state.turret_mode = TurretState::AIM;
            des_turret_state.yaw_angle = jetson_state.desired_yaw_rads * (180 / M_PI);
            des_turret_state.pitch_angle = jetson_state.desired_pitch_rads * (180 / M_PI);
        } else {
            chassis.setWheelPower({0, 0, 0, 0});
            des_turret_state.turret_mode = TurretState::SLEEP;
        }

        // Shooter Logic
        if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP) {
            des_shoot_state = ShootState::SHOOT;
        } else if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID) {
            if (jetson_state.shoot_status == 0) {
                des_shoot_state = ShootState::FLYWHEEL;
            } else {
                des_shoot_state = ShootState::SHOOT;
            }
        } else {
            des_shoot_state = ShootState::OFF;
        }

        // printf("time %ld", us_ticker_read());

        turret.setState(des_turret_state);
        shooter.setState(des_shoot_state);

        turret.periodic(chassis.getChassisSpeeds().vOmega * 60 / (2 * PI));
        chassis.periodic(&imuAngles);
        shooter.periodic(referee.power_heat_data.shooter_17mm_1_barrel_heat,
                         referee.robot_status.shooter_barrel_heat_limit);

        // jetson comms
        stm_state.game_state = 4;
        stm_state.robot_hp = 200;

        stm_state.chassis_x_velocity = chassis.getChassisSpeeds().vX;
        stm_state.chassis_y_velocity = chassis.getChassisSpeeds().vY;
        stm_state.chassis_rotation = chassis.getChassisSpeeds().vOmega;

        stm_state.yaw_angle_rads = degreesToRadians(turret.getState().yaw_angle);
        stm_state.yaw_velocity = degreesToRadians(turret.getState().yaw_velo);
        stm_state.pitch_angle_rads = degreesToRadians(turret.getState().pitch_angle);
        stm_state.pitch_velocity = degreesToRadians(turret.getState().pitch_angle);
        jetson.write(stm_state);

        // printf("time %ld", us_ticker_read());

        // Debug print statements
        // printf("des: %.2f, %.2f, %.2f %d \n", jetson.read().desired_x_vel,
        // jetson.read().desired_y_vel, jetson.read().desired_angular_vel,
        // jetson.read().localization_calibration); printf("y: %.2f\n",
        // turret.getState().yaw_angle); printf("p: %.2f\n",
        // turret.getState().pitch_angle + remote_.getPitch()); printf("p:
        // %.2f\n", turret.getState().pitch_angle); printf("%d\n",
        // shooter.getState()); printf("v:%d\n",testmot>>VELOCITY); printf("cx:
        // %.2f\n", remote_.getChassisX()); printf("switch: %d\n",
        // remote_.getSwitch(Remote::Switch::RIGHT_SWITCH)); printf("imu:
        // %.2f\n", imu.getImuAngles().yaw);
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop
};

DigitalOut led0 = PC_1;

int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{};
    Sentry Sentry(config);

    Sentry.main_loop();
    // blocking
}