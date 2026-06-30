#include "base_robot/BaseRobot.h"
#include "util/algorithms/general_functions.h"

#include "subsystems/OmniWheelSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"

#include "util/communications/CANHandler.h"
#include "util/communications/DJIRemote2.h"
#include "util/communications/jetson/Jetson.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"
#include "util/peripherals/encoder/MA4.h"
#include "util/peripherals/imu/ISM330.h"

#include <algorithm>
#include <pinmap.h>
#include <us_ticker_api.h>
#include <us_ticker_defines.h>

constexpr auto IMU_I2C_SDA = PB_7;
constexpr auto IMU_I2C_SCL = PB_8;
constexpr auto IMU_RESET = PA_8;

constexpr float PITCH_LOWER_BOUND{-22.0};
constexpr float PITCH_UPPER_BOUND{25.0};

constexpr float JOYSTICK_YAW_SENSITIVITY_DPS = 600;
constexpr float JOYSTICK_PITCH_SENSITIVITY_DPS = 300;
// Mouse sensitivity initialized
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

constexpr PID::config YAW_VEL_PID     = {181, 3.655 * 10e-3, 4.51 * 7.5, 32000, 1000};
constexpr PID::config YAW_POS_PID     = {1, 0, 0, 45, 2};
const float yaw_static_friction       = 0;//-150;       // We multiply it by dir
const float yaw_kinetic_friction      = 0;       // We multiply this by yawvelo

constexpr PID::config PITCH_VEL_PID   = {173.8994, 4.898 * 10e-6, 12.474 * 10e3, 16000, 2000}; //{25, 0.001, 5, 16000, 1000};
constexpr PID::config PITCH_POS_PID   = {1, 0, 0,30,2}; //{1, 0, 0, 30, 2};
const float pitch_gravity_feedforward = -1200;    // We multiply this by cos(angle)
const float pitch_static_friction     = 0;//635.0 / 2;       // We multiply it by dir
const float pitch_kinetic_friction    = 0; //5.5;     // We multiply this by pitchvelo

constexpr PID::config FL_VEL_CONFIG = {2.58, 0.23 * 1e-3, 17.3 * 1e-3}; //original p was 2.58
constexpr PID::config FR_VEL_CONFIG = {2.75, 0.574 * 1e-3, 17.9 * 1e-3}; //original p was 2.75
constexpr PID::config BL_VEL_CONFIG = {4.1, 0.0523 * 1e-3, 10.9 * 1e-3}; //original p was 4.1
constexpr PID::config BR_VEL_CONFIG = {3.9, 0.159 * 1e-3, 26.1 * 1e-3}; //original p was 3.9 

constexpr PID::config FLYWHEEL_L_PID = {7.1849, 0.000042634, 0};
constexpr PID::config FLYWHEEL_R_PID = {7.1849, 0.000042634, 0};
constexpr PID::config INDEXER_PID_VEL = {2.7, 0.001, 0};
constexpr PID::config INDEXER_PID_POS = {0.1, 0, 0.001};


// Config variables
TurretSubsystem::config turret_config = {
    7,
    M3508,
    8,
    M3508,
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
    2,
    4,
    6,
    FLYWHEEL_L_PID,
    FLYWHEEL_R_PID,
    INDEXER_PID_VEL,
    INDEXER_PID_POS,
    CANHandler::CANBUS_2,
    false
};
OmniWheelSubsystem::Config chassis_config = {
    1,      // left_front_can_id
    3,      // right_front_can_id
    4,      // left_back_can_id
    2,      // right_back_can_id
    FL_VEL_CONFIG,
    FR_VEL_CONFIG,
    BL_VEL_CONFIG,
    BR_VEL_CONFIG,
    0.51,  // radius
    // 208
    188,     // yaw_initial_offset_ticks
    120,
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
    ISM330 imu_;
    MA4 encoder_;
    BufferedSerial jetson_raw_serial;
    Jetson jetson;

    Jetson::WriteState stm_state;
    Jetson::ReadState jetson_state;

    TurretSubsystem turret_;
    ShooterSubsystem shooter_;
    OmniWheelSubsystem chassis_;

    bool imu_initialized{false};

    Infantry(Config &config)
        : BaseRobot(config),
        // clang-format off
        i2c_(IMU_I2C_SDA, IMU_I2C_SCL), 
        imu_(i2c_, 0x6B),
        encoder_(PB_4),
        jetson_raw_serial(PC_12, PD_2,115200),
        jetson(jetson_raw_serial),
        turret_(turret_config, imu_),
        shooter_(shooter_config),
        chassis_(chassis_config, &encoder_)
        // clang-format on
    {
        pin_mode(IMU_I2C_SCL, PinMode::OpenDrainPullUp);
        pin_mode(IMU_I2C_SDA, PinMode::OpenDrainPullUp);
    }

    ~Infantry() {}

    void init() override {
        timer = us_ticker_read();
        imu_.begin(0.9, 0);
    }

    void periodic(unsigned long dt_us) override {
        // TODO this should be threaded inside imu instead
        imu_.mahonyUpdateIMU(dt_us / 1000000.0);
        imuAngles = imu_.getImuAngles();
        // TODO: use this in code correctly to drive faster
        // chassis_.power_limit = referee_.robot_status.chassis_power_limit;
        max_linear_vel = 1.24 + 0.0513 * chassis_.power_limit +
                         0.000216 * (chassis_.power_limit * chassis_.power_limit);
        // max_linear_vel = 5.0;
        des_chassis_state.vX = jy * max_linear_vel;
        des_chassis_state.vY = -jx * max_linear_vel;

        // Read jetson
        jetson_state = jetson.read();
        // check if new jetson state given and if we want cv
        if (cv_enabled_ && (us_ticker_read() - jetson_state.stamp_us ) / 1000 < 500) {
            yaw_desired_angle = jetson_state.desired_yaw_rads;
            pitch_desired_angle = jetson_state.desired_pitch_rads;
        }

        // Turret from remote
        yaw_desired_angle -= myaw * MOUSE_SENSITIVITY_YAW_DPS * dt_us / 1000000;
        yaw_desired_angle -= jyaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_us / 1000000;
        yaw_desired_angle = capAngle(yaw_desired_angle);
        des_turret_state.yaw_angle_degs = yaw_desired_angle;

        pitch_desired_angle -= mpitch * MOUSE_SENSITIVITY_PITCH_DPS * dt_us / 1000000;
        pitch_desired_angle -= jpitch * JOYSTICK_PITCH_SENSITIVITY_DPS * dt_us / 1000000;
        pitch_desired_angle = std::clamp(pitch_desired_angle, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);
        des_turret_state.pitch_angle_degs = pitch_desired_angle;

        // Chassis logic
        if (drive == 'u' || (drive == 'o' && remote_.getMode() == DJIRemote2::ModeSwitch::MODE_S)) {
            des_chassis_state.vOmega = 0;
            chassis_.setChassisSpeeds(des_chassis_state, OmniWheelSubsystem::YAW_ORIENTED);
            des_turret_state.turret_mode = TurretState::AIM;
            referee_.is_aligned = false;
            referee_.is_cv_on = false;
            referee_.is_spinning = false;
        } // else if (drive == 'd' || commented out for inspection, later just get rid of mode C - Dil 
        //            (drive == 'o' &&
        //             remote_.getMode() == DJIRemote2::ModeSwitch::MODE_C)) {
        //     // des_chassis_state.vOmega = omega_speed;
        //     chassis_.setChassisSpeeds(des_chassis_state, OmniWheelSubsystem::BEYBLADE);
        //     des_turret_state.turret_mode = TurretState::AIM;
        //     referee_.is_aligned = false;
        //     referee_.is_cv_on = false;
        //     referee_.is_spinning = true;
        // } 
        else 
        {
            chassis_.setChassisSpeeds({0, 0, 0});
            des_turret_state.turret_mode = TurretState::SLEEP;
            des_turret_state.yaw_angle_degs = turret_.getState().yaw_angle_degs;
            yaw_desired_angle = turret_.getState().yaw_angle_degs;
            des_turret_state.pitch_angle_degs = 0;
            referee_.is_aligned = true;
            referee_.is_cv_on = true;
            referee_.is_spinning = false;
        }

        // Shooter Logic 
        //REMOVED remote_.PAUSEToggled() == true && FROM THE FIRST CONDITION
        if ((remote_.PAUSEToggled() == true && remote_.TriggerPressed() == true) || remote_.getMouseL()) {
            des_shoot_state = ShootState::SHOOT;
            referee_.is_flywheel_on = true;
        } else if ((remote_.CUSTRPressed() == true || remote_.keyPressed(DJIRemote2::Key::Z)) && remote_.PAUSEToggled() == true) { //Make sure flywheel is on since that's part 
            des_shoot_state = ShootState::JAM;
        } else if (remote_.PAUSEToggled() == true || shot == 'd') {
            des_shoot_state = ShootState::FLYWHEEL;
            referee_.is_flywheel_on = true;
        } else {
            des_shoot_state = ShootState::OFF;
            referee_.is_flywheel_on = false;
        }

        turret_.setState(des_turret_state);
        shooter_.setState(des_shoot_state);

        turret_.periodic(chassis_.getChassisSpeeds().vOmega * 60 / (2 * PI));
        chassis_.power_limit = referee_.robot_status.chassis_power_limit;
        chassis_.periodic(imuAngles);
        shooter_.periodic(referee_.power_heat_data.shooter_17mm_1_barrel_heat,
                         referee_.robot_status.shooter_barrel_heat_limit);

        // jetson comms
        set_jetson_state();
        jetson.write(stm_state);

        // printf("time %.4f\n", dt_us / 1000000.0);

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
        // printf("%d\n", referee_.get_game_progress());
        // printf("yp %.2f \n", encoder_.encoderMovingAverage());
        // printf("%.2f, %.2f, %.2f\n", imuAngles.roll, imuAngles.pitch, imuAngles.yaw);
        // ChassisSpeeds speeds = chassis_.getChassisSpeeds();
        // printf("whoosh: %.3f\n", speeds.vOmega);
        // printf("ref inspd: %.2f\n", referee_.shoot_data.initial_speed);
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop

    void set_jetson_state() {
        stm_state.activate_CV = cv_enabled_;
        stm_state.game_state = referee_.get_game_progress();
        stm_state.robot_hp = referee_.get_remain_hp();
        stm_state.team_color = referee_.is_red_or_blue();

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
    // blocking
}