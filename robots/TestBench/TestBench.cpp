#include "base_robot/BaseRobot.h"
#include "subsystems/ChassisSubsystem.h"
#include "util/communications/CANHandler.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"

#include <algorithm>


constexpr auto IMU_I2C_SDA = PB_7;
constexpr auto IMU_I2C_SCL = PB_8;
constexpr auto IMU_RESET   = PA_8;

constexpr PID::config FL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config FR_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BL_VEL_CONFIG = {3, 0, 0};
constexpr PID::config BR_VEL_CONFIG = {3, 0, 0};


ChassisSpeeds des_chassis_state;


class Infantry : public BaseRobot {
public:
    I2C    i2c_;
    BNO055 imu_;

    ChassisSubsystem chassis;

    bool imu_initialized{false};

    Infantry(Config& config)
        : BaseRobot(config),
          i2c_(IMU_I2C_SDA, IMU_I2C_SCL),
          imu_(i2c_, IMU_RESET, MODE_IMU),
          chassis(ChassisSubsystem::Config{
              1,        // left_front_can_id
              2,        // right_front_can_id
              3,        // left_back_can_id
              4,        // right_back_can_id
              0.22617,  // radius
              0.065,    // speed_pid_ff_ks
              nullptr,  // no yaw motor
              0,        // yaw_initial_offset_ticks
              imu_}) {}

    ~Infantry() {}

    void init() override {}

    void periodic(unsigned long dt_us) override {
        
        IMU::EulerAngles imuAngles = imu_.read();

        if (!imu_initialized) {
            IMU::EulerAngles angles = imu_.getImuAngles();
            if (angles.pitch == 0.0 && angles.yaw == 0.0 && angles.roll == 0.0) {
                return;
            }
            imu_initialized = true;
        }

        max_linear_vel = 1.24 + 0.0513 * chassis.power_limit + 0.000216 * (chassis.power_limit * chassis.power_limit);

        des_chassis_state.vX = jy * max_linear_vel;
        des_chassis_state.vY = jx * max_linear_vel;
        des_chassis_state.vOmega = omega_speed;  

        if (drive == 'u' ||
            (drive == 'o' &&
             remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) ==
                 Remote::SwitchState::UP)) {

            des_chassis_state.vOmega = 0;

            chassis.setChassisSpeeds(
                des_chassis_state,
                ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);

        } else if (drive == 'd' ||
                   (drive == 'o' &&
                    remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) ==
                        Remote::SwitchState::DOWN)) {

            des_chassis_state.vOmega = omega_speed;

            chassis.setChassisSpeeds(
                des_chassis_state,
                ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);

        } else {
            chassis.setWheelPower({0, 0, 0, 0});
        }

        chassis.periodic(&imuAngles);
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override {
        return 2; // 500 Hz
    }
};


int main() {
    printf("HELLO\n");

    BaseRobot::Config config{};
    Infantry infantry(config);

    infantry.main_loop();
}