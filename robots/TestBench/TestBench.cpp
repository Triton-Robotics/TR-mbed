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
#include <string.h>


uint8_t tempByte;

PID::config test_motor_vel_PID = {1, 0, 0};
PID::config test_motor_pos_PID = {1, 0, 0};

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

class TestBench : public BaseRobot {
  public:
	uint8_t rxBuffer[50]; 
	// initialize a variable to keep track of whether the header has been found
	int headerFound = 0;
	// declare pin number (bufferedserial)
	BufferedSerial pc;  // TX, RX, baud

    TestBench(Config &config)
        : BaseRobot(config),
          // clang-format off
		pc(PC_10, PC_11, 921600)
        // clang-format on        
    {}

    ~TestBench() {}

    void init() override 
    {
        
    }
    
    void periodic(unsigned long dt_us) override {
		// logic goes here
		if (pc.readable()){
			pc.read(rxBuffer, sizeof(rxBuffer));
			printf("Received data: %s\n", rxBuffer);
		}
        printf("Hello\n");
		for(unsigned int i = 0; i <= sizeof(rxBuffer); i++) {
			if((rxBuffer[i] == 0xA9) || (rxBuffer[i] == 0x53)) {
				headerFound = 1;
				printf("Found header at index %d\n", i);
				// find the next 20 bytes and print them out 
				// if the header is 0xA9, the next 20 bytes are the data
				// if the header is 0x53, the next 20 bytes are the data
				// print out the data in hex format
				// for example, if the header is 0xA9, print "Data: " followed by the 20 bytes in hex format
				// if the header is 0x53, print "Data: " followed by the 20 bytes in hex format
				// if the header is not found, print "Header not found" after the loop
				if (i + 20 < sizeof(rxBuffer)) {
                	printf("Data: ");
                	for (size_t j = 1; j <= 20; j++) {
                   		printf("%02X ", rxBuffer[i + j]);
                	}
                	printf("\n");
            	} 
				else {
                printf("Not enough bytes after header to read 20 bytes\n");
            	}

				break; // Exit the loop after finding the header
			}
			else {
				headerFound = 0;
				printf("No header found!\n");
			}
				
		}
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 15; } // 500 Hz loop
};


int main() {
    //printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{}; 
    TestBench TestBench(config);
	
	

    TestBench.main_loop();
    // blocking
}