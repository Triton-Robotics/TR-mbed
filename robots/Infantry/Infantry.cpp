#include "base_robot/BaseRobot.h"
// #include "subsystems/ChassisSubsystem.h"
#include "subsystems/OmniWheelSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "util/communications/CANHandler.h"

#define I2C_SDA PB_7
#define I2C_SCL PB_8
#define IMU_RESET PA_8

constexpr int pitch_zero_offset_ticks = 1500;

I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);

// TODO: put acc values
PID::config yaw_vel_PID;
PID::config yaw_pos_PID;
PID::config pitch_vel_PID;
PID::config pitch_pos_PID;
TurretSubsystem turret;

PID::config fl_vel_config;
PID::config fr_vel_config;
PID::config bl_vel_config;
PID::config br_vel_config;
OmniWheelSubsystem chassis;

// TODO: put acc values
PID::config flywheelL_PID;
PID::config flywheelR_PID;
PID::config indexer_PID_vel;
PID::config indexer_PID_pos;
ShooterSubsystem shooter;

class Infantry : public BaseRobot {
public:
    Infantry() : BaseRobot(config_) {}

    // default config
    const Config config_ = Config();

    ~Infantry() {}

    void init() override {
        TurretSubsystem::config turret_cfg =
        {
            4,
            7,
            pitch_zero_offset_ticks,
            yaw_vel_PID,
            yaw_pos_PID,
            pitch_vel_PID,
            pitch_pos_PID,
            CANHandler::CANBUS_1,
            CANHandler::CANBUS_2,
            &imu
        };
        turret = TurretSubsystem(turret_cfg);

        OmniWheelSubsystem::config chassis_cfg =
        {
            1,
            2,
            3,
            4,
            0.22617,
            referee.robot_status.chassis_power_limit,
            2.92,
            100,
            0.065 * INT16_MAX,
            4.8,
            fl_vel_config,
            fr_vel_config,
            bl_vel_config,
            br_vel_config,
            CANHandler::CANBUS_1,
            &imu,
            &turret,
            6500,
            0 // TODO add correct yaw_align
        };
        chassis = OmniWheelSubsystem(chassis_cfg);

        ShooterSubsystem::config shooter_cfg =
        {
            ShooterSubsystem::BURST,
            referee.robot_status.shooter_barrel_heat_limit,
            1,
            2,
            7,
            flywheelL_PID,
            flywheelR_PID,
            indexer_PID_vel,
            indexer_PID_pos,
            CANHandler::CANBUS_2,
        };
        shooter = ShooterSubsystem(shooter_cfg);
    }

    void periodic(unsigned long dt_us) override {
      // TODO: use remote to setState for all subsystems

      imu.read();

      turret.periodic();
      chassis.periodic();
      shooter.periodic();
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop
};

int main() {
    Infantry infantry;

    // blocking
    infantry.main_loop();
}