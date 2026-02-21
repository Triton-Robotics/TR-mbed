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

PID::config test_motor_vel_PID = {1, 0, 0};
PID::config test_motor_pos_PID = {1, 0, 0};
PwmIn encoder_(PA_7);
double movavg = 0.0;


class TestBench : public BaseRobot {
  public:
    // DJIMotor motor;

    TestBench(Config &config)
        : BaseRobot(config)
          // clang-format off
        // motor(DJIMotor::config{
        //     1,
        //     CANHandler::CANBUS_1,
        //     M3508,
        //     "Test motor",
        //     test_motor_vel_PID,
        //     test_motor_pos_PID
        // })
        // clang-format on        
    {}

    ~TestBench() {}

    void init() override 
    {
        
    }

    
    double getEncoderYawPosition() {
        static float filtered_yaw = 0.0f;
        float filter_alpha = 0.2f;  // 0.0-1.0: lower = more smoothing, higher = more responsive

        float duty_raw = encoder_.dutycycle();
        float duty_min = 0.02943f;   // 2.943%
        float duty_max = 0.97058f;   // 97.058%
        double yaw_position = (double)(abs(((duty_raw - duty_min) / (duty_max - duty_min)) * 360.0));
        filtered_yaw = filtered_yaw * (1.0f - filter_alpha) + yaw_position *  filter_alpha;
        // printf("%.2f\n",yaw_position);
        return filtered_yaw;
    }

    double encoderMovingAverage() {
        const int windowSize = 20;
        static double readings[windowSize] = {0};
        static int index = 0;
        static bool filled = false;

        double newReading = getEncoderYawPosition();

        readings[index] = newReading;
        index = (index + 1) % windowSize;
        if (index == 0) {
            filled = true;
        }

        double sum = 0.0;
        double result = 0.0;

        if (filled) {
            for (int i = 0; i < windowSize; i++) {
                sum += readings[i];
            }
            result = sum / windowSize;
        } else {
            for (int i = 0; i < index; i++) {
                sum += readings[i];
            }
            result = sum / index;
        }
        // printf("%.2f\n",result);
        return result;

    }

        
    void periodic(unsigned long dt_us) override {
        
        movavg = encoderMovingAverage();
        printf("%.2f\n",movavg);
        fflush(stdout);

    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop
};


int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{}; 
    TestBench TestBench(config);

    TestBench.main_loop();
    // blocking
}