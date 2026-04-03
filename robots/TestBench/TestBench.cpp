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
const float V_REF = 3.3f; // Reference voltage for AnalogIn
const float SENSOR_VCC = 5.0f; // Voltage supply for ACS712 sensor
const float SENSITIVITY = 0.185f; //185mV/A for 5A model
const float KT_M2006 = 0.18f; // Torque constant for M2006 motor, in Nm/A
const float KT_M3508 = 0.30f; // Torque constant for M3508 motor, in Nm/A
bool safety_tripped = false;

class TestBench : public BaseRobot {
  public:
    DJIMotor motor;
    //Declare AnalogIn for current sensing
    mbed::AnalogIn ain;

//Varibales for current sensing
float current_amps = 0.0;
float torque_nm = 0.0; 
float filtered_current = 0.0;
float calibrated_offset = 2.5f; // Initial guess for offset voltage, will be calibrated in init()

    TestBench(Config &config)
        : BaseRobot(config),
          // clang-format off
        motor(DJIMotor::config{
            4,
            CANHandler::CANBUS_2,
            M3508, 
            "Test motor",
            test_motor_vel_PID,
            test_motor_pos_PID
        }), ain(PA_7)
        // clang-format on        
    {}

    //Torque Calculation From ACS712 Current Sensor
    float calculateTorque(float amps) {
        return amps * KT_M3508;
    }

    ~TestBench() {}

    void init() override 
    {
        printf("Calibrating ACS712...keep motor stationary.\n");
        // Calibrate the ACS712 by taking multiple readings and averaging them to find the offset voltage
        float sum = 0;
        int samples = 200;
        for (int i = 0; i < samples; i++) {
            sum += (ain.read() * V_REF); // Convert reading to voltage
            ThisThread::sleep_for(10ms);
    }
    
    calibrated_offset = sum / samples;
    printf("Offset voltage: %.2f V\n", calibrated_offset);

    // Print headers for the spreadsheet
    printf("\nVoltage\tCurrent\tTorque\n");

    }
    
    void periodic(unsigned long dt_us) override {
    // 1. Check Safety Latch First
    if (safety_tripped) {
        motor.setPower(0); 
        // We don't return here yet because we still want to see the 
        // Current/Voltage readings even if the motor is killed.
    } 
    else {
        // Remote Control Logic
        if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP) {
            motor.setPower(800);
        } 
        else if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID) {
            motor.setPower(500);
        } 
        else {
            motor.setPower(0);
        }
    }

    // 3. Sensor Calculations
    float voltage = ain.read() * V_REF;
    current_amps = (voltage - calibrated_offset) / SENSITIVITY;

    // 4. Low Pass Filter (The "Phase Lag" implementation!)
    float alpha = 0.1f; 
    filtered_current = alpha * current_amps + (1.0f - alpha) * filtered_current;
    
    float display_current = filtered_current;

    // 5. Update Safety Latch
    if (abs(display_current) > 4.8f) { 
        safety_tripped = true;
        motor.setPower(0);
        printf("!!! SAFETY TRIGGERED: %.2f A !!!\n", display_current);
        return; // Exit this loop immediately
    }

    // 6. Deadzone & Torque
    if(abs(display_current) < 0.12f) { 
        display_current = 0.0f;
    }
    torque_nm = calculateTorque(display_current);

    // 7. Spreadsheet-Ready Printing
    static int count = 0;
    if (count++ % 100 == 0) { 
        printf("%.3f\t%.2f\t%.4f\n", voltage, display_current, torque_nm);
    }
}

    void end_of_loop() override {}
    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop
};


int main() {
    ThisThread::sleep_for(2s); // Wait for everything to initialize
    
    printf("HELLO - TestBench Starting...\n");
    BaseRobot::Config config = BaseRobot::Config{}; 
    TestBench robot(config);

    robot.main_loop();
    // blocking
}