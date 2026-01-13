#include "I2C.h"
#include "ThisThread.h"
#include "base_robot/BaseRobot.h"
// #include "subsystems/ChassisSubsystem.h"
#include "subsystems/OmniWheelSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "util/communications/CANHandler.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"
#include <memory>

constexpr auto IMU_I2C_SDA = PB_7;
constexpr auto IMU_I2C_SCL = PB_8;
constexpr auto IMU_RESET = PA_8;

constexpr int pitch_zero_offset_ticks = 1500;


// TODO: put acc values
PID::config yaw_vel_PID = {708.1461, 4.721, 2.6555, 32000, 8000};
PID::config yaw_pos_PID = {1.18, 0, 0, 90, 2};
PID::config pitch_vel_PID = {500,0.8,0, 32000, 2000};
PID::config pitch_pos_PID = {1.5,0.0005,0.05, 30, 2};

// TurretSubsystem::config turret_config {
//     4,
//     GM6020,
//     7,
//     GM6020,
//     pitch_zero_offset_ticks,
//     yaw_vel_PID,
//     yaw_pos_PID,
//     pitch_vel_PID,
//     pitch_pos_PID,
//     CANHandler::CANBUS_1,
//     CANHandler::CANBUS_2,
//     imu_ptr.get()
// };


// TurretSubsystem turret(turret_config);

PID::config fl_vel_config = {3, 0, 0};
PID::config fr_vel_config = {3, 0, 0};
PID::config bl_vel_config = {3, 0, 0};
PID::config br_vel_config = {3, 0, 0};
// OmniWheelSubsystem chassis({
//     1,
//     2,
//     3,
//     4,
//     0.22617,
//     60,
//     2.92,
//     100,
//     0.065 * INT16_MAX,
//     4.8,
//     fl_vel_config,
//     fr_vel_config,
//     bl_vel_config,
//     br_vel_config,
//     CANHandler::CANBUS_1,
//     imu_ptr.get(),
//     &turret,
//     6500,
//     0 // TODO add correct yaw_align
// });

// // TODO: put acc values
PID::config flywheelL_PID = {7.1849, 0.000042634, 0};
PID::config flywheelR_PID = {7.1849, 0.000042634, 0};
PID::config indexer_PID_vel = {2.7, 0.001, 0};
PID::config indexer_PID_pos = {0.1,0,0.001};
ShooterSubsystem shooter({
    ShooterSubsystem::BURST,
    0,
    1,
    2,
    7,
    flywheelL_PID,
    flywheelR_PID,
    indexer_PID_vel,
    indexer_PID_pos,
    CANHandler::CANBUS_2
});

// State variables
OmniWheelSubsystem::ChassisState des_chassis_state;
TurretState des_turret_state;
ShootState des_shoot_state;


// DJIMotor::config flyLcfg =
// {
//     1,
//     CANHandler::CANBUS_2,
//     M3508,
//     "Right Flywheel",
//     flywheelL_PID
// };
// DJIMotor testmot(flyLcfg);

int remoteTimer = 0;

class Infantry : public BaseRobot {
public:

    I2C i2c_;
    BNO055 imu_;

    TurretSubsystem::config turret_config; 
    TurretSubsystem turret;

    OmniWheelSubsystem::config chassis_config;
    OmniWheelSubsystem chassis;

    Infantry(Config &config) : 
        BaseRobot(config),     
    i2c_(IMU_I2C_SDA, IMU_I2C_SCL),
    imu_(i2c_, IMU_RESET, MODE_IMU),
    turret_config{
        4,
        GM6020,
        7,
        GM6020,
        pitch_zero_offset_ticks,
        yaw_vel_PID,
        yaw_pos_PID,
        pitch_vel_PID,
        pitch_pos_PID,
        CANHandler::CANBUS_1,
        CANHandler::CANBUS_2,
        -1,
        imu_
    },
    turret(turret_config),

    chassis_config{
    1,
    2,
    3,
    4,
    0.22617,
    60,
    2.92,
    100,
    0.065 * INT16_MAX,
    4.8,
    fl_vel_config,
    fr_vel_config,
    bl_vel_config,
    br_vel_config,
    CANHandler::CANBUS_1,
    imu_,
    turret,
    6500,
    0 // TODO add correct yaw_align
    },
    chassis(chassis_config)
    {}

    ~Infantry() {}

    void init() override 
    {
        // TODO: better way to do this
        // chassis.setPowerLimit(referee.robot_status.chassis_power_limit);
        shooter.setHeatLimit(referee.robot_status.shooter_barrel_heat_limit);
        printf("init\n");
    }
    
    void periodic(unsigned long dt_us) override {
        // TODO: use remote to setState for all subsystems
        if (remoteTimer > 20) {
            remoteTimer = 0;
            remote_.read();
        }
        remoteTimer += 1;

        // TODO this should be threaded inside imu instead
        IMU::EulerAngles imuAngles = imu_.read();
        // printf("%.2f", imuAngles.yaw);
        
        // Chassis + Turret Logic
        // des_chassis_state.vel.vX = remote_.getChassisX();
        // des_chassis_state.vel.vY = remote_.getChassisY();
        // printf("remote %d\n", remote_.getSwitch(Remote::Switch::RIGHT_SWITCH));
        // printf("remotel %d\n", remote_.getSwitch(Remote::Switch::LEFT_SWITCH));
        if (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP)
        {
            // des_chassis_state.mode = OmniWheelSubsystem::ChassisMode::YAW_ORIENTED;
            // des_chassis_state.vel.vOmega = 0;

            des_turret_state = TurretState::AIM;
        }
        else if (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN)
        {
            // des_chassis_state.mode = OmniWheelSubsystem::ChassisMode::BEYBLADE;
            des_turret_state = TurretState::AIM;
        }
        else
        {
            // des_chassis_state.mode = OmniWheelSubsystem::ChassisMode::OFF;
            des_turret_state = TurretState::SLEEP;
        }
        // chassis.setChassisState(des_chassis_state);
        
        // Set turret state
        turret.setState(des_turret_state);
        // turret.set_desired_turret(turret.getState().yaw_angle + remote_.getYaw(), remote_.getPitch(), chassis.getChassisState().vel.vOmega);
        turret.set_desired_turret(turret.getState().yaw_angle, remote_.getPitch(), 0);
        // printf("curr: %.2f, des: %.2f\n", turret.getState().pitch_angle, 0.0);

        // Shooter Logic
        if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP)
        {
            des_shoot_state = ShootState::SHOOT;
            // testmot.setPower(0);
        }
        else if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID)
        {
            des_shoot_state = ShootState::FLYWHEEL;
            // testmot.setPower(1000);
        }
        else
        {
            des_shoot_state = ShootState::OFF;
        }
        // shooter.setState(des_shoot_state, referee.power_heat_data.shooter_17mm_1_barrel_heat);
        
        turret.periodic();
        // chassis.periodic();
        // shooter.periodic();


        // Debug print statements
        // printf("y: %.2f\n", turret.getState().yaw_angle);
        // printf("p: %.2f\n", turret.getState().pitch_angle + remote_.getPitch());
        // printf("p: %.2f\n", turret.getState().pitch_angle);
        // printf("%d\n", shooter.getState());
        // printf("v:%d\n",testmot>>VELOCITY);
        // printf("cx: %.2f\n", remote_.getChassisX());
        // printf("switch: %d\n", remote_.getSwitch(Remote::Switch::RIGHT_SWITCH));
        // printf("imu: %.2f\n", imu.getImuAngles().yaw);
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop
};

DigitalOut led0 = PC_1;

int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{}; 
    Infantry infantry(config);

    infantry.main_loop();
    // blocking
}