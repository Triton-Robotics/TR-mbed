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

#include "mbed.h"

PID::config test_motor_vel_PID = {1, 0, 0};
PID::config test_motor_pos_PID = {1, 0, 0};

//Constants for ACS712 and M2006 motor
const float SENSITIVITY = 0.185; //185mV/A for 5A model
const float V_REF = 3.3;
const float KT_M2006 = 0.18; // Torque constant for M2006 motor, in Nm/A

class TestBench : public BaseRobot {
  public:
    DJIMotor motor;
    //Declare AnalogIn for current sensing
    AnalogIn ain;

//Varibales for current sensing
float current_amps = 0.0;
float torque_nm = 0.0; 
float calibrated_offset = 1.65; // Initial guess for offset voltage, will be calibrated in init() 

    TestBench(Config &config)
        : BaseRobot(config),
          // clang-format off
        motor(DJIMotor::config{
            1,
            CANHandler::CANBUS_1,
            M2006, 
            "Test motor",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        ain(PA_7)
        // clang-format on        
    {}

    //Torque Calculation From ACS712 Current Sensor
    float calculateTorque(float amps) {
        return amps * KT_M2006;
    }

    ~TestBench() {}

    void init() override 
    {
        printf("Calibrating ACS712...don't draw curent.\n");
        // Calibrate the ACS712 by taking multiple readings and averaging them to find the offset voltage
        float sum = 0;
        int samples = 100;
        for (int i = 0; i < samples; i++) {
            sum += ain.read() * V_REF;
            ThisThread::sleep_for(10ms);
    }
    
    calibrated_offset = sum / samples;
    printf("Offset voltage: %.2f V\n", calibrated_offset);

    }
    
    void periodic(unsigned long dt_us) override {
        
        //Calculate Voltage from AnalogIn reading
        float voltage = ain.read() * V_REF;

        //Calculate Current from Voltage and Sensitivity
        current_amps = (voltage - calibrated_offset) / SENSITIVITY; // Subtract offset voltage (1.65V for 0A)

        //Calculate Torque from Current
        torque_nm = calculateTorque(current_amps);

        // Print current and torque values
        printf("Current: %.2f A, Torque: %.4f Nm\n", current_amps, torque_nm);

        //
        motor.setPower(0);
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