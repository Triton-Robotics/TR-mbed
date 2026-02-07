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
// Mouse sensitivity initialized
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
    // M_PI is a common constant for Pi in cmath
    return degrees * M_PI / 180.0;
}

// TODOS make example directory with simple examples of how to use motors /
// subsystems as reference for testbench

class Infantry : public BaseRobot {
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

    // Remote control variables
    float scalar = 1;
    float jx = 0; // -1 to 1
    float jy = 0; // -1 to 1
    // Pitch, Yaw
    float jpitch = 0; // -1 to 1
    float jyaw = 0; // -1 to 1
    float myaw = 0;
    float mpitch = 0;
    int pitchVelo = 0;
    // joystick tolerance
    float tolerance = 0.05;
    // Keyboard Driving
    float mult = 0.7;
    float omega_speed = 0;
    float max_linear_vel = 0;

    // GENERAL VARIABLES
    // drive and shooting mode
    char drive = 'o'; //default o when using joystick
    char shot = 'o'; //default o when using joystick
    bool cv_enabled = false;


    bool imu_initialized{false};

    Infantry(Config &config)
        : BaseRobot(config),
          // clang-format off
        i2c_(IMU_I2C_SDA, IMU_I2C_SCL), 
        imu_(i2c_, IMU_RESET, MODE_IMU),
        jetson_raw_serial(PC_12, PD_2,115200), // TODO: check higher baud to see if still works
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
            CANHandler::CANBUS_1,
            CANHandler::CANBUS_2,
            -1,
            imu_,
            PITCH_LOWER_BOUND,
            PITCH_UPPER_BOUND
        }    
        ),

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
            false
        }),

        // TODO add passing in individual PID objects for the motors
        chassis(ChassisSubsystem::Config{
            1,      // left_front_can_id
            2,      // right_front_can_id
            3,      // left_back_can_id
            4,      // right_back_can_id
            0.22617,  // radius
            0.065,    // speed_pid_ff_ks
            &turret.yaw,  // yaw_motor
            356,     // yaw_initial_offset_ticks
            imu_     
        }
        )
    // clang-format on
    {}

    ~Infantry() {}

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

        remoteRead();

        // des_chassis_state.vX = remote_.getChannel(Remote::Channel::LEFT_VERTICAL);
        // des_chassis_state.vY = -1 * remote_.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        des_chassis_state.vX = jy;
        des_chassis_state.vY = -1 * jx;

        // Turret from remote
        // float joystick_yaw = remote_.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
        yaw_desired_angle -= myaw * MOUSE_SENSITIVITY_YAW_DPS * dt_us / 1000000;
        yaw_desired_angle -= jyaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_us / 1000000;
        yaw_desired_angle = capAngle(yaw_desired_angle);
        des_turret_state.yaw_angle = yaw_desired_angle;

        // float joystick_pitch = remote_.getChannel(Remote::Channel::RIGHT_VERTICAL);
        pitch_desired_angle -= mpitch * MOUSE_SENSITIVITY_PITCH_DPS * dt_us / 1000000;
        pitch_desired_angle += jpitch * JOYSTICK_PITCH_SENSITIVITY_DPS * dt_us / 1000000;
        pitch_desired_angle = std::clamp(pitch_desired_angle, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);
        des_turret_state.pitch_angle = pitch_desired_angle;

        jetson_state = jetson.read();

        if (drive == 'u' || (drive =='o' && remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP)) {
            // TODO: think about how we want to implement jetson aiming
            // des_turret_state.pitch_angle = jetson_state.desired_pitch_rads;
            // des_turret_state.yaw_angle = jetson_state.desired_yaw_rads;

            des_chassis_state.vOmega = 0;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state.turret_mode = TurretState::AIM;
        } else if (drive == 'd' || (drive =='o' && remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN)) {
            des_chassis_state.vOmega = omega_speed;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::YAW_ORIENTED);
            des_turret_state.turret_mode = TurretState::AIM;
        } else {
            chassis.setWheelPower({0, 0, 0, 0});
            des_turret_state.turret_mode = TurretState::SLEEP;
        }

        // Shooter Logic
        if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP) {
            des_shoot_state = ShootState::SHOOT;
        } else if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID) {
            des_shoot_state = ShootState::FLYWHEEL;
        } else {
            des_shoot_state = ShootState::OFF;
        }

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

    void remoteRead()
    {
        //Keyboard-based drive and shoot mode
        if(remote_.keyPressed(Remote::Key::R)){
            drive = 'm';
        }else if(remote_.keyPressed(Remote::Key::E)){
            drive = 'u';
        }else if(remote_.keyPressed(Remote::Key::Q)){
            drive = 'd';        
        }
        if(remote_.keyPressed(Remote::Key::V)){
            shot = 'm';
        }else if(remote_.keyPressed(Remote::Key::C)){
            shot = 'd';        
        }
        
        if(remote_.getMouseR() || remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID){
            cv_enabled = true;
        }else if(!remote_.getMouseR() ){
            cv_enabled = false;
        }

        //Driving input
        scalar = 1;
        jx = remote_.leftX() * scalar; // -1 to 1
        jy = remote_.leftY() * scalar; // -1 to 1
        //Pitch, Yaw
        jpitch = remote_.rightY() * scalar; // -1 to 1
        jyaw = remote_.rightX() * scalar; // -1 to 1

        myaw = remote_.getMouseX();
        mpitch = -remote_.getMouseY();

        jx = (abs(jx) < tolerance) ? 0 : jx;
        jy = (abs(jy) < tolerance) ? 0 : jy;
        jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
        jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
        

        // Shift to make robot go slower
        if (remote_.keyPressed(Remote::Key::SHIFT)) {
            mult = 0.5;
        }
        if(remote_.keyPressed(Remote::Key::CTRL)){
            mult = 1;
        }

        jx += mult * ((remote_.keyPressed(Remote::Key::D) ? 1 : 0) + (remote_.keyPressed(Remote::Key::A) ? -1 : 0));
        jy += mult * ((remote_.keyPressed(Remote::Key::W) ? 1 : 0) + (remote_.keyPressed(Remote::Key::S) ? -1 : 0));

        float j_hypo = sqrt(jx * jx + jy * jy);
        if(j_hypo > 1.0){
            jx = jx / j_hypo;
            jy = jy / j_hypo;
        }
        //Bounding the four j variables
        jx = max(-1.0F, min(1.0F, jx));
        jy = max(-1.0F, min(1.0F, jy));
        jpitch = max(-1.0F, min(1.0F, jpitch));
        jyaw = max(-1.0F, min(1.0F, jyaw));

        max_linear_vel = -1.24 + 0.0513 * chassis.power_limit + -0.000216 * (chassis.power_limit * chassis.power_limit);
        // float max_omega = 0.326 + 0.0857 * chassis_power_limit + -0.000183 * (chassis_power_limit * chassis_power_limit);
        float max_omega = 4.8;

        if(remote_.keyPressed(Remote::Key::CTRL)){
            jx = 0.0;
            jy = 0.0;
            max_omega = 6.1;
        }

        float linear_hypo = sqrtf(jx * jx + jy * jy);
        if(linear_hypo > 0.8){
            linear_hypo = 0.8;
        }

        float available_beyblade = 1.0 - linear_hypo;
        omega_speed = max_omega * available_beyblade;
    }
};

int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{};
    Infantry infantry(config);

    infantry.main_loop();
    // blocking
}