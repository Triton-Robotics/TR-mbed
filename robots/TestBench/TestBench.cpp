#include "base_robot/BaseRobot.h"
#include "util/algorithms/general_functions.h"

#include "subsystems/ChassisSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"

#include "util/communications/CANHandler.h"
#include "util/communications/jetson/Jetson.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"

#include "util/communications/DJIRemote2.h"

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

// State variables
ChassisSpeeds des_chassis_state;
TurretSubsystem::TurretInfo des_turret_state;
ShootState des_shoot_state;

int remoteTimer = 0;

float pitch_desired_angle = 0.0;
float yaw_desired_angle = 0.0;

VTMInput in = {};


class TestBench : public BaseRobot {
public:
    I2C i2c_;
    BNO055 imu_;
    DJIRemote2 vtm;
    TurretSubsystem turret;
    ChassisSubsystem chassis;


    TestBench(Config &config)
        : BaseRobot(config),
          i2c_(IMU_I2C_SDA, IMU_I2C_SCL), 
          imu_(i2c_, IMU_RESET, MODE_IMU),
          vtm(PB_6, PA_10, 921600),
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
        })
    {}

    ~TestBench() {}

    void init() override
    {
    }

    void periodic(unsigned long dt_us) override
    {   
        if (vtm.update()) {
            in = vtm.getData();
            // printf("Frame dt = %llu us (%.2f Hz)\n",
            //        vtm.getFramePeriodUs(),
            //        vtm.getFrameRateHz());
            
            static int timer = 0;
            if (timer++ % 5 == 0) {
                printf("ch0=%u ch1=%u ch2=%u ch3=%u mode=%u pause=%u btnL=%u btnR=%u dial=%u trigger=%u "
                    "mX=%d mY=%d mZ=%d mL=%u mR=%u mM=%u kb=%u CRC=%u\n",
                    in.ch0, in.ch1, in.ch2, in.ch3,
                    in.mode, in.pause, in.btnL, in.btnR,
                    in.dial, in.trigger,
                    in.mouseX, in.mouseY, in.mouseZ,
                    in.mouseL, in.mouseR, in.mouseM,
                    in.keyboard, in.CRC_in);
            }
        }

        jx = (in.ch3 - 1024) / 660.0;
        jy = (in.ch2 - 1024) / 660.0;
        jyaw = (in.ch0 - 1024) / 660.0;
        jpitch = (in.ch1 - 1024) / 660.0;


        jx = (abs(jx) < tolerance) ? 0 : jx;
        jy = (abs(jy) < tolerance) ? 0 : jy;
        jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
        jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;

        //Bounding the four j variables
        jx = max(-1.0F, min(1.0F, jx));
        jy = max(-1.0F, min(1.0F, jy));
        jpitch = max(-1.0F, min(1.0F, jpitch));
        jyaw = max(-1.0F, min(1.0F, jyaw));
        
        printf("%.2f,%.2f,%.2f,%.2f\n",jx,jy,jpitch,jyaw);

        // TODO: use this in code correctly to drive faster
        max_linear_vel = 1.24 + 0.0513 * chassis.power_limit + 0.000216 * (chassis.power_limit * chassis.power_limit);
        des_chassis_state.vX = jy * max_linear_vel;
        des_chassis_state.vY = -1 * jx * max_linear_vel;

        printf("%.2f, %.2f, %.2f\n",des_chassis_state.vX,des_chassis_state.vY,des_chassis_state.vOmega);


        // Turret from remote
        yaw_desired_angle -= myaw * MOUSE_SENSITIVITY_YAW_DPS * dt_us / 1000000;
        yaw_desired_angle -= jyaw * JOYSTICK_YAW_SENSITIVITY_DPS * dt_us / 1000000;
        yaw_desired_angle = capAngle(yaw_desired_angle);
        des_turret_state.yaw_angle_degs = yaw_desired_angle;

        pitch_desired_angle -= mpitch * MOUSE_SENSITIVITY_PITCH_DPS * dt_us / 1000000;
        pitch_desired_angle += jpitch * JOYSTICK_PITCH_SENSITIVITY_DPS * dt_us / 1000000;
        pitch_desired_angle = std::clamp(pitch_desired_angle, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);
        des_turret_state.pitch_angle_degs = pitch_desired_angle;




        if (in.mode == 2) {
            // TODO: think about how we want to implement jetson aiming
            // des_turret_state.pitch_angle = jetson_state.desired_pitch_rads;
            // des_turret_state.yaw_angle = jetson_state.desired_yaw_rads;
            printf("state2\n");
            omega_speed = 2;
            des_chassis_state.vOmega = -jyaw*omega_speed;;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);
            des_turret_state.turret_mode = TurretState::AIM;
        } else if (in.mode == 0) {
            printf("state0\n");
            des_chassis_state.vOmega = 2;
            chassis.setChassisSpeeds(des_chassis_state, ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);
            des_turret_state.turret_mode = TurretState::AIM;
        } else {
            printf("state1\n");
            chassis.setWheelPower({0, 0, 0, 0});
            des_turret_state.turret_mode = TurretState::SLEEP;
        }


    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 15; }
};

int main() {
    BaseRobot::Config config = BaseRobot::Config{};
    TestBench testBench(config);
    testBench.main_loop();
}