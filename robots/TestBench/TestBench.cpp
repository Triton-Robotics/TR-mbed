#include "base_robot/BaseRobot.h"
#include "util/algorithms/general_functions.h"

#include "subsystems/ChassisSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"

#include "util/communications/CANHandler.h"
#include "util/communications/jetson/Jetson.h"
#include "util/motor/DJIMotor.h"
#include "util/peripherals/imu/BNO055.h"

#include <Timer.h>
#include <algorithm>
#include <us_ticker_defines.h>

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


float voltage = 0.0f;
float display_current = 0.0f;
float torque_nm = 0.0f;
int output_power = 0;
unsigned long curr_time = 0;
int current_counter = 0;

class TestBench : public BaseRobot {
  public:
    DJIMotor motor1, motor2, motor3, motor4;
    //Declare AnalogIn for current sensing
    mbed::AnalogIn ain;

//Varibales for current sensing
float current_amps = 0.0;
//float torque_nm = 0.0; 
float filtered_current = 0.0;
float calibrated_offset = 2.5f; // Initial guess for offset voltage, will be calibrated in init()

    TestBench(Config &config)
        : BaseRobot(config),
          // clang-format off
        motor1(DJIMotor::config{
            1,
            CANHandler::CANBUS_2,
            M3508, 
            "Test motor",
            test_motor_vel_PID,
            test_motor_pos_PID
        }), motor2(DJIMotor::config{
            2,
            CANHandler::CANBUS_2,
            M3508, 
            "Test motor",
            test_motor_vel_PID,
            test_motor_pos_PID
        }), motor3(DJIMotor::config{
            4,
            CANHandler::CANBUS_2,
            M3508, 
            "Test motor",
            test_motor_vel_PID,
            test_motor_pos_PID
        }), motor4(DJIMotor::config{
            3,
            CANHandler::CANBUS_2,
            M3508, 
            "Test motor",
            test_motor_vel_PID,
            test_motor_pos_PID
        }), 
        ain(PA_7)
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

    curr_time = us_ticker_read(); // Initialize current time for remote control logic
    }
    
    void periodic(unsigned long dt_us) override {
    // 1. Check Safety Latch First
    if (safety_tripped) {
        motor1.setPower(0); 
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        // We don't return here yet because we still want to see the 
        // Current/Voltage readings even if the motor is killed.
    } 
    else {
        // Remote Control Logic
        if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP) {
            // if (curr_time - us_ticker_read() > 10000000) { // Every 1 second
            //     curr_time = us_ticker_read();
            //     if (output_power < 8000) {
            //         output_power = output_power + 100;
            //     }
            // }
            motor1.setPower(output_power), 
            motor2.setPower(output_power),
            motor3.setPower(output_power),
            motor4.setPower(output_power);
            current_counter++;
            if (current_counter > 100 && output_power <8001){
                output_power+=50;
                current_counter = 0;
            }

        } 
        else if (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID) {
            motor1.setPower(500);
            motor2.setPower(500);
            motor3.setPower(500);
            motor4.setPower(500);
        } 
        else {
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);

            output_power = 0; // Reset instead of flashing
        }
    }

    // 3. Sensor Calculations
    voltage = ain.read() * V_REF;
    current_amps = (voltage - calibrated_offset) / SENSITIVITY;

    // 4. Low Pass Filter (The "Phase Lag" implementation!)
    float alpha = 0.1f; 
    filtered_current = alpha * current_amps + (1.0f - alpha) * filtered_current;
    
    display_current = filtered_current;

    // 5. Update Safety Latch
    if (abs(display_current) > 4.8f) { 
        safety_tripped = true;
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
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
        printf("%d\t%.2f\t%.4f\n", output_power, display_current, torque_nm);
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