#include "base_robot/BaseRobot.h"
#include "util/algorithms/general_functions.h"

#include "subsystems/ChassisSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"

#include "util/communications/jetson/Jetson.h"
#include "util/peripherals/imu/BNO055.h"

constexpr auto IMU_I2C_SDA = PB_7;
constexpr auto IMU_I2C_SCL = PB_8;
constexpr auto IMU_RESET = PA_8;

constexpr int pitch_zero_offset_ticks = 1500;

constexpr float JOYSTICK_YAW_SENSITIVITY_DPS = 600;
constexpr float JOYSTICK_PITCH_SENSITIVITY_DPS = 300;


// TODO: put acc values
PID::config yaw_vel_PID = {350, 0.5, 2.5, 32000, 2000};
PID::config yaw_pos_PID = {0.5, 0, 0, 90, 2};
PID::config pitch_vel_PID = {500,0.8,0, 32000, 2000};
PID::config pitch_pos_PID = {1.5,0.0005,0.05, 30, 2};


PID::config fl_vel_config = {3, 0, 0};
PID::config fr_vel_config = {3, 0, 0};
PID::config bl_vel_config = {3, 0, 0};
PID::config br_vel_config = {3, 0, 0};

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
ChassisSpeeds des_chassis_state;
TurretState des_turret_state;
ShootState des_shoot_state;

int remoteTimer = 0;

float pitch_desired_angle = 0.0;
float yaw_desired_angle = 0.0;

class Infantry : public BaseRobot {
public:

    I2C i2c_;
    BNO055 imu_;

    // TODO: put the BufferedSerial inside Jetson (idk if we wanna do that tho for SPI)
    BufferedSerial jetson_;
    Jetson jetson;

    Jetson::WriteState stm_state;
    Jetson::ReadState jetson_state;

    TurretSubsystem::config turret_config; 
    TurretSubsystem turret;

    ChassisSubsystem::Config chassis_config;
    ChassisSubsystem chassis;

    Infantry(Config &config) : 
        BaseRobot(config),     
    i2c_(IMU_I2C_SDA, IMU_I2C_SCL),
    imu_(i2c_, IMU_RESET, MODE_IMU),
    jetson_(PC_12, PD_2, 115200), // TODO: check higher baud to see if still works
    jetson(jetson_),
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
        1,      // left_front_can_id
        2,      // right_front_can_id
        3,      // left_back_can_id
        4,      // right_back_can_id
        0.22617,  // radius
        0.065,    // speed_pid_ff_ks
        &turret.yaw,  // yaw_motor
        6500,     // yaw_initial_offset_ticks
        imu_     
    },

    chassis(chassis_config)
    {}

    ~Infantry() {}

    void init() override 
    {
        // TODO: better way to do this
        // chassis.setPowerLimit(referee.robot_status.chassis_power_limit);
        // chassis.setSpeedFF_Ks(0.065);
        // chassis.setYawReference(&turret.yaw, 6500);
        // shooter.setHeatLimit(referee.robot_status.shooter_barrel_heat_limit);
        printf("init\n");

        turret.periodic();
        pitch_desired_angle = 0.0;
        yaw_desired_angle = imu_.read().yaw;
    }
    
    void periodic(unsigned long dt_us) override {
        // TODO: use remote to setState for all subsystems
        if (remoteTimer > 10) {
            remoteTimer = 0;
            remote_.read();
        }
        remoteTimer += 1;

        // TODO: fill out state update
        // stm_state.game_state = 4;
        // stm_state.robot_hp = 60;
        // jetson.write(stm_state);


        // TODO this should be threaded inside imu instead
        IMU::EulerAngles imuAngles = imu_.read();
        // printf("%.2f", imuAngles.yaw);
        
        // Chassis + Turret Logic
        des_chassis_state.vX = remote_.getChassisX();
        des_chassis_state.vY = remote_.getChassisY();
        // printf("remote %d\n", remote_.getSwitch(Remote::Switch::RIGHT_SWITCH));
        // printf("remotel %d\n", remote_.getSwitch(Remote::Switch::LEFT_SWITCH));
        if (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP)
        {
            des_chassis_state.vOmega = 0;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state = TurretState::AIM;
        }
        else if (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN)
        {
            des_chassis_state.vOmega = 4.8;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state = TurretState::AIM;
        }
        else
        {
            chassis.setWheelPower({0, 0, 0, 0});
            des_turret_state = TurretState::SLEEP;
        }
        
        // Set turret state
        turret.setState(des_turret_state);

        float joystick_yaw = remote_.getChannel(Remote::Channel::RIGHT_HORIZONTAL);

        yaw_desired_angle -= joystick_yaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_us / 1000000;
        // normalize between [-180, 180)
        // TODO yaw pid does not properly respect the discontinuity at 180, -180 it goes the long way around instead
        yaw_desired_angle = capAngle(yaw_desired_angle);


        float joystick_pitch = remote_.getChannel(Remote::Channel::RIGHT_VERTICAL);

        // TODO need to limit this to between lower and upper bound of pitch
        pitch_desired_angle += joystick_pitch * JOYSTICK_PITCH_SENSITIVITY_DPS * dt_us / 1000000;

        turret.set_desired_turret(yaw_desired_angle, pitch_desired_angle, chassis.getChassisSpeeds().vOmega * 60 / (2*PI));

        // Shooter Logic
        if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP)
        {
            des_shoot_state = ShootState::SHOOT;
        }
        else if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID)
        {
            des_shoot_state = ShootState::FLYWHEEL;
        }
        else
        {
            des_shoot_state = ShootState::OFF;
        }
        // shooter.setState(des_shoot_state, referee.power_heat_data.shooter_17mm_1_barrel_heat);
        
        // printf("time %ld", us_ticker_read());

        turret.periodic();
        chassis.periodic(&imuAngles);
        // shooter.periodic();

        printf("test");

        // printf("time %ld", us_ticker_read());

        // Debug print statements
        // printf("des: %.2f, %.2f, %.2f %d \n", jetson.read().desired_x_vel, jetson.read().desired_y_vel, jetson.read().desired_angular_vel, jetson.read().localization_calibration);
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