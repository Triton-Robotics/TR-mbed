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

#include "main.h"
#include <string.h>

UART_HandleTypeDef huart2;
uint8_t rxBuffer[256]; 
uint8_t tempByte;

PID::config test_motor_vel_PID = {1, 0, 0};
PID::config test_motor_pos_PID = {1, 0, 0};

// declare pin number (bufferedserial)
BufferedSerial pc(PA_2, PA_3, 115200);  // TX, RX, baud

class TestBench : public BaseRobot {
  public:
    DJIMotor motor;

    TestBench(Config &config)
        : BaseRobot(config),
          // clang-format off
        motor(DJIMotor::config{
            1,
            CANHandler::CANBUS_1,
            M3508,
            "Test motor",
            test_motor_vel_PID,
            test_motor_pos_PID
        })
        // clang-format on        
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

struct VTMinput {
	uint16_t ch0, ch1, ch2, ch3; // 11-bit
	uint8_t  mode;               // 2 bits
	uint8_t  pause, btnL, btnR;  // 1 bit
	uint16_t dial;               // 11-bit
	uint8_t  trigger;            // 1 bit
	int16_t  mouseX, mouseY, mouseZ; // 16-bit 
	uint8_t  mouseL, mouseR, mouseM; // 2-bit 
	uint16_t keyboard;           // 16-bit
};

static framereading(){
	//find header 1 and header 2
	while(0xA9) {
		
	}
}

int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{}; 
    TestBench TestBench(config);
	//do everything here (including the code for packets and all that)
	

    TestBench.main_loop();
    // blocking
}