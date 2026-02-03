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

class TestBench : public BaseRobot {
public:

    DJIMotor motor;

    TestBench(Config &config) : 
        BaseRobot(config),
        motor(DJIMotor::config{
            1,
            CANHandler::CANBUS_1,
            M3508,
            "Test motor",
            test_motor_vel_PID,
            test_motor_pos_PID
        })
    {}

    ~TestBench() {}

    void init() override 
    {
        
    }
    
    void periodic(unsigned long dt_us) override {
        motor.setPower(0);
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop
};

DigitalOut led0 = PC_1;

int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{}; 
    TestBench TestBench(config);

    TestBench.main_loop();
    // blocking
}