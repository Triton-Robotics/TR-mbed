#include "base_robot/BaseRobot.h"
#include "subsystems/ChassisSubsystem.h"
#include "util/communications/CANHandler.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"
#include "util/peripherals/encoder/MA4.h"

#include <algorithm>

constexpr auto IMU_I2C_SDA = PB_7;
constexpr auto IMU_I2C_SCL = PB_8;
constexpr auto IMU_RESET   = PA_8;

// Sweep table — identical to TestBench.cpp
struct TestStep {
    float vX;
    float vY;
    float vW;
    int duration_loops;
    const char* label;
};

static constexpr float MV = 4.345f;
static constexpr float D  = MV / sqrtf(2.0f);  // ~3.072 m/s
static constexpr int   H  = 250;                // loops per step (~500ms at 500Hz)

static const TestStep SWEEP[] = {

    // EXPERIMENT 1: Pure spin — vX=0, vY=0, vary vW
    { 0, 0, 0.0f, H, "E1 idle"      },
    { 0, 0, 0.3f, H, "E1 vW=0.3"   },
    { 0, 0, 0.6f, H, "E1 vW=0.6"   },
    { 0, 0, 0.9f, H, "E1 vW=0.9"   },
    { 0, 0, 1.2f, H, "E1 vW=1.2"   },
    { 0, 0, 1.5f, H, "E1 vW=1.5"   },
    { 0, 0, 2.0f, H, "E1 vW=2.0"   },
    { 0, 0, 2.5f, H, "E1 vW=2.5"   },
    { 0, 0, 3.0f, H, "E1 vW=3.0"   },
    { 0, 0, 3.5f, H, "E1 vW=3.5"   },
    { 0, 0, 4.0f, H, "E1 vW=4.0"   },
    { 0, 0, 4.5f, H, "E1 vW=4.5"   },
    { 0, 0, 4.8f, H, "E1 vW=4.8"   },
    { 0, 0, 5.2f, H, "E1 vW=5.2"   },
    { 0, 0, 5.6f, H, "E1 vW=5.6"   },
    { 0, 0, 6.0f, H, "E1 vW=6.0"   },
    { 0, 0, 6.1f, H, "E1 vW=6.1"   },
    { 0, 0, 0.0f, H, "E1 idle end" },

    // EXPERIMENT 2: Pure axial X — vY=0, vW=0, vary vX
    { 0.0f, 0, 0, H, "E2 vX=0.0"   },
    { 0.3f, 0, 0, H, "E2 vX=0.3"   },
    { 0.6f, 0, 0, H, "E2 vX=0.6"   },
    { 0.9f, 0, 0, H, "E2 vX=0.9"   },
    { 1.2f, 0, 0, H, "E2 vX=1.2"   },
    { 1.5f, 0, 0, H, "E2 vX=1.5"   },
    { 1.8f, 0, 0, H, "E2 vX=1.8"   },
    { 2.2f, 0, 0, H, "E2 vX=2.2"   },
    { 2.6f, 0, 0, H, "E2 vX=2.6"   },
    { 3.0f, 0, 0, H, "E2 vX=3.0"   },
    { 3.3f, 0, 0, H, "E2 vX=3.3"   },
    { 3.6f, 0, 0, H, "E2 vX=3.6"   },
    { 3.9f, 0, 0, H, "E2 vX=3.9"   },
    { 4.1f, 0, 0, H, "E2 vX=4.1"   },
    { 4.2f, 0, 0, H, "E2 vX=4.2"   },
    { MV,   0, 0, H, "E2 vX=4.345" },
    { 0,    0, 0, H, "E2 idle end" },

    // EXPERIMENT 3: Pure axial Y — vX=0, vW=0, vary vY (symmetry check)
    { 0, 0.3f, 0, H, "E3 vY=0.3"   },
    { 0, 0.6f, 0, H, "E3 vY=0.6"   },
    { 0, 0.9f, 0, H, "E3 vY=0.9"   },
    { 0, 1.2f, 0, H, "E3 vY=1.2"   },
    { 0, 1.5f, 0, H, "E3 vY=1.5"   },
    { 0, 2.0f, 0, H, "E3 vY=2.0"   },
    { 0, 2.5f, 0, H, "E3 vY=2.5"   },
    { 0, 3.0f, 0, H, "E3 vY=3.0"   },
    { 0, 3.5f, 0, H, "E3 vY=3.5"   },
    { 0, 4.0f, 0, H, "E3 vY=4.0"   },
    { 0, 4.2f, 0, H, "E3 vY=4.2"   },
    { 0, MV,   0, H, "E3 vY=4.345" },
    { 0, 0,    0, H, "E3 idle end" },

    // EXPERIMENT 4: Pure diagonal — vX=vY, vW=0, vary magnitude
    { 0.21f, 0.21f, 0, H, "E4 diag|v|=0.3"   },
    { 0.42f, 0.42f, 0, H, "E4 diag|v|=0.6"   },
    { 0.64f, 0.64f, 0, H, "E4 diag|v|=0.9"   },
    { 0.85f, 0.85f, 0, H, "E4 diag|v|=1.2"   },
    { 1.06f, 1.06f, 0, H, "E4 diag|v|=1.5"   },
    { 1.27f, 1.27f, 0, H, "E4 diag|v|=1.8"   },
    { 1.56f, 1.56f, 0, H, "E4 diag|v|=2.2"   },
    { 1.84f, 1.84f, 0, H, "E4 diag|v|=2.6"   },
    { 2.12f, 2.12f, 0, H, "E4 diag|v|=3.0"   },
    { 2.33f, 2.33f, 0, H, "E4 diag|v|=3.3"   },
    { 2.55f, 2.55f, 0, H, "E4 diag|v|=3.6"   },
    { 2.76f, 2.76f, 0, H, "E4 diag|v|=3.9"   },
    { 2.90f, 2.90f, 0, H, "E4 diag|v|=4.1"   },
    { 2.97f, 2.97f, 0, H, "E4 diag|v|=4.2"   },
    { D,     D,     0, H, "E4 diag|v|=4.345" },
    { 0,     0,     0, H, "E4 idle end"       },

    // EXPERIMENT 5: Direction sweep — fixed |vXY|=4.345, vary theta, vW=0
    { MV*1.000f, MV*0.000f, 0, H, "E5 theta=0"    },
    { MV*0.991f, MV*0.131f, 0, H, "E5 theta=7.5"  },
    { MV*0.966f, MV*0.259f, 0, H, "E5 theta=15"   },
    { MV*0.924f, MV*0.383f, 0, H, "E5 theta=22.5" },
    { MV*0.866f, MV*0.500f, 0, H, "E5 theta=30"   },
    { MV*0.793f, MV*0.609f, 0, H, "E5 theta=37.5" },
    { D,         D,         0, H, "E5 theta=45"   },
    { MV*0.609f, MV*0.793f, 0, H, "E5 theta=52.5" },
    { MV*0.500f, MV*0.866f, 0, H, "E5 theta=60"   },
    { MV*0.383f, MV*0.924f, 0, H, "E5 theta=67.5" },
    { MV*0.259f, MV*0.966f, 0, H, "E5 theta=75"   },
    { MV*0.131f, MV*0.991f, 0, H, "E5 theta=82.5" },
    { MV*0.000f, MV*1.000f, 0, H, "E5 theta=90"   },
    { 0,         0,         0, H, "E5 idle end"    },

    // EXPERIMENT 6: Combined axial — fixed vW=3.0, vary vX (vY=0)
    { 0,    0, 3.0f, H, "E6 vW=3 vX=0"     },
    { 0.3f, 0, 3.0f, H, "E6 vW=3 vX=0.3"   },
    { 0.6f, 0, 3.0f, H, "E6 vW=3 vX=0.6"   },
    { 1.0f, 0, 3.0f, H, "E6 vW=3 vX=1.0"   },
    { 1.4f, 0, 3.0f, H, "E6 vW=3 vX=1.4"   },
    { 1.8f, 0, 3.0f, H, "E6 vW=3 vX=1.8"   },
    { 2.2f, 0, 3.0f, H, "E6 vW=3 vX=2.2"   },
    { 2.6f, 0, 3.0f, H, "E6 vW=3 vX=2.6"   },
    { 3.0f, 0, 3.0f, H, "E6 vW=3 vX=3.0"   },
    { 3.3f, 0, 3.0f, H, "E6 vW=3 vX=3.3"   },
    { 3.6f, 0, 3.0f, H, "E6 vW=3 vX=3.6"   },
    { 3.9f, 0, 3.0f, H, "E6 vW=3 vX=3.9"   },
    { 4.1f, 0, 3.0f, H, "E6 vW=3 vX=4.1"   },
    { 4.2f, 0, 3.0f, H, "E6 vW=3 vX=4.2"   },
    { MV,   0, 3.0f, H, "E6 vW=3 vX=4.345" },
    { 0,    0, 0,    H, "E6 idle end"       },

    // EXPERIMENT 7: Combined diagonal — fixed vW=3.0, vary |vXY| diagonally
    { 0,     0,     3.0f, H, "E7 vW=3 diag=0"     },
    { 0.21f, 0.21f, 3.0f, H, "E7 vW=3 diag=0.3"   },
    { 0.42f, 0.42f, 3.0f, H, "E7 vW=3 diag=0.6"   },
    { 0.64f, 0.64f, 3.0f, H, "E7 vW=3 diag=0.9"   },
    { 1.06f, 1.06f, 3.0f, H, "E7 vW=3 diag=1.5"   },
    { 1.41f, 1.41f, 3.0f, H, "E7 vW=3 diag=2.0"   },
    { 1.77f, 1.77f, 3.0f, H, "E7 vW=3 diag=2.5"   },
    { 2.12f, 2.12f, 3.0f, H, "E7 vW=3 diag=3.0"   },
    { 2.47f, 2.47f, 3.0f, H, "E7 vW=3 diag=3.5"   },
    { 2.76f, 2.76f, 3.0f, H, "E7 vW=3 diag=3.9"   },
    { 2.90f, 2.90f, 3.0f, H, "E7 vW=3 diag=4.1"   },
    { 2.97f, 2.97f, 3.0f, H, "E7 vW=3 diag=4.2"   },
    { D,     D,     3.0f, H, "E7 vW=3 diag=4.345" },
    { 0,     0,     0,    H, "E7 idle end"         },
};

static const int N_STEPS = sizeof(SWEEP) / sizeof(SWEEP[0]);

// ----------------------------------------------------------------------------
// State
// ----------------------------------------------------------------------------
ChassisSpeeds des_chassis_state;
IMU::EulerAngles imuAngles;

// ----------------------------------------------------------------------------
// InfantrySweep — real infantry hardware config, sweep control loop
// ----------------------------------------------------------------------------
class Infantry : public BaseRobot {
public:
    I2C    i2c_;
    BNO055 imu_;
    MA4    encoder_;

    ChassisSubsystem chassis_;

    bool imu_initialized{false};
    int  current_step{0};
    int  step_loop_count{0};
    bool sweep_done{false};

    Infantry(Config& config)
        : BaseRobot(config),
          i2c_(IMU_I2C_SDA, IMU_I2C_SCL),
          imu_(i2c_, IMU_RESET, MODE_IMU),
          encoder_(PA_7),
          chassis_(ChassisSubsystem::Config{
              1,          // left_front_can_id
              3,          // right_front_can_id
              6,          // left_back_can_id
              2,          // right_back_can_id
              0.22617f,   // radius
              0.065f,     // speed_pid_ff_ks
              nullptr,    // no yaw motor during sweep
              168.3f + 90.0f,  // yaw_initial_offset_ticks
              imu_,
              &encoder_
          })
    {}

    ~Infantry() {}

    void init() override {
        printf("# InfantrySweep starting — %d steps\n", N_STEPS);
    }

    void periodic(unsigned long dt_us) override {
        imuAngles = imu_.read();

        if (!imu_initialized) {
            IMU::EulerAngles angles = imu_.getImuAngles();
            if (angles.pitch == 0.0 && angles.yaw == 0.0 && angles.roll == 0.0) {
                return;
            }
            imu_initialized = true;
        }

        // Sweep done — hold still
        if (sweep_done) {
            des_chassis_state = {0, 0, 0};
            chassis_.setChassisSpeeds(des_chassis_state,
                                      ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);
            chassis_.periodic(&imuAngles);
            return;
        }

        if (step_loop_count == 0) {
            const TestStep& s = SWEEP[current_step];
            printf("# STEP %d/%d: %s  vX=%.3f vY=%.3f vW=%.3f\n",
                   current_step + 1, N_STEPS,
                   s.label, s.vX, s.vY, s.vW);
        }

        const TestStep& step = SWEEP[current_step];
        des_chassis_state.vX     = step.vX;
        des_chassis_state.vY     = step.vY;
        des_chassis_state.vOmega = step.vW;

        chassis_.setChassisSpeeds(des_chassis_state,
                                  ChassisSubsystem::DRIVE_MODE::ROBOT_ORIENTED);
        chassis_.periodic(&imuAngles);

        // Advance step counter
        if (++step_loop_count >= step.duration_loops) {
            step_loop_count = 0;
            if (++current_step >= N_STEPS) {
                sweep_done = true;
                printf("# SWEEP COMPLETE\n");
            }
        }
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz
};

int main() {
    printf("HELLO\n");

    BaseRobot::Config config{};
    Infantry infantry(config);

    infantry.main_loop();
}