#include "ThisThread.h"
#include "base_robot/BaseRobot.h"
// #include "subsystems/ChassisSubsystem.h"
#include "subsystems/OmniWheelSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "util/communications/CANHandler.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"

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
TurretSubsystem turret({
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
});

PID::config fl_vel_config = {3, 0, 0};
PID::config fr_vel_config = {3, 0, 0};
PID::config bl_vel_config = {3, 0, 0};
PID::config br_vel_config = {3, 0, 0};
OmniWheelSubsystem chassis({
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
    &imu,
    &turret,
    6500,
    0 // TODO add correct yaw_align
});

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


class Infantry : public BaseRobot {
public:
    Infantry(Config &config) : BaseRobot(config) {}

    // default config
    // inline static Config config_ = Config();

    ~Infantry() {}

    void init() override 
    {
        // TODO: better way to do this
        chassis.setPowerLimit(referee.robot_status.chassis_power_limit);
        shooter.setHeatLimit(referee.robot_status.shooter_barrel_heat_limit);
        // printf("init\n");
    }
    
    void periodic(unsigned long dt_us) override {
        // TODO: use remote to setState for all subsystems
        remote_.read();
        imu.read();

        
        // Chassis + Turret Logic
        des_chassis_state.vel.vX = remote_.getChassisX();
        des_chassis_state.vel.vY = remote_.getChassisY();
        if (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP)
        {
            des_chassis_state.mode = OmniWheelSubsystem::ChassisMode::YAW_ORIENTED;
            des_chassis_state.vel.vOmega = 0;

            des_turret_state = TurretState::AIM;
        }
        else if (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN)
        {
            des_chassis_state.mode = OmniWheelSubsystem::ChassisMode::BEYBLADE;
            des_turret_state = TurretState::AIM;
        }
        else
        {
            des_chassis_state.mode = OmniWheelSubsystem::ChassisMode::OFF;
            des_turret_state = TurretState::SLEEP;
        }
        // chassis.setChassisState(des_chassis_state);
        
        // Set turret state
        turret.set_desired_turret(remote_.getYaw(), remote_.getPitch(), chassis.getChassisState().vel.vOmega);
        
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
        printf("%d\n", shooter.getState());
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