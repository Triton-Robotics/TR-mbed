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

// testing (not used anymore)
// uint8_t tempByte;

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
	uint8_t rxBuffer[21]; 
	uint16_t ch0 = 0; 
	uint16_t ch1 = 0; 
	uint16_t ch2 = 0; 
	uint16_t ch3 = 0; 
	uint8_t mode = 0;
	uint8_t pause = 0;
	uint8_t btnL = 0;
	uint8_t btnR = 0;
	uint16_t dial = 0;
	uint8_t trigger = 0;
	int16_t mouseX = 0;
	int16_t mouseY = 0;
	int16_t mouseZ = 0;
	uint8_t mouseL = 0;
	uint8_t mouseR = 0;
	uint8_t mouseM = 0;
	uint16_t keyboard = 0;
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
		}
		
		for(unsigned int i = 0; i < sizeof(rxBuffer); i++) {
			if((rxBuffer[i] == 0xA9) && (rxBuffer[i+1] == 0x53)) {
				headerFound = 1;
				//debug purposes only
				//printf("%x %x %x %x %x %x %x\n",rxBuffer[i],rxBuffer[i+1],rxBuffer[i+2],rxBuffer[i+3],rxBuffer[i+4],rxBuffer[i+5],rxBuffer[i+6]);

				ch0 = ((uint16_t)(rxBuffer[i+2])) | ((uint16_t)(rxBuffer[i+3] & 0x07) << 8); // 11 bits for ch0
				ch1 = ((((uint16_t)rxBuffer[i+3]) >> 3) & 0x1F) | ((((uint16_t)rxBuffer[i+4]) & 0x3F) << 5); // 11 bits for ch1
				ch2 = (((uint16_t)rxBuffer[i+4] >> 6) & 0x03) | (((uint16_t)rxBuffer[i+5]) << 2) | ((((uint16_t)rxBuffer[i+6]) & 0x01) << 10); // 11 bits for ch2
				ch3 = ((((uint16_t)rxBuffer[i+6]) >> 1) & 0x7F) | ((((uint16_t)rxBuffer[i+7]) & 0x0F) << 7); // 11 bits for ch3

				mode = (rxBuffer[i+7] >> 4) & 0x03; // 2 bits for mode

				pause = (rxBuffer[i+7] >> 6) & 0x01; // 1 bit for pause

				btnL = (rxBuffer[i+7] >> 7) & 0x01; // 1 bit for btnL
				btnR = (rxBuffer[i+8]) & 0x01;	// 1 bit for btnR

				dial = ((((uint16_t)rxBuffer[i+8]) >> 1) & 0x7F) | ((((uint16_t)rxBuffer[i+9]) & 0x0F) << 7); // 11 bits for dial

				trigger = (rxBuffer[i+9] >> 4) & 0x01; // 1 bit for trigger

				mouseX = (int16_t)((((uint16_t)rxBuffer[i+9]  >> 5) & 0x07) | (((uint16_t)rxBuffer[i+10]) << 3) | ((((uint16_t)rxBuffer[i+11]) & 0x1F) << 11)); // 16 bits for mouseX
				mouseY = (int16_t)((((uint16_t)rxBuffer[i+11] >> 5) & 0x07) | (((uint16_t)rxBuffer[i+12]) << 3) | ((((uint16_t)rxBuffer[i+13]) & 0x1F) << 11)); // 16 bits for mouseY
				mouseZ = (int16_t)((((uint16_t)rxBuffer[i+13] >> 5) & 0x07) | (((uint16_t)rxBuffer[i+14]) << 3) | ((((uint16_t)rxBuffer[i+15]) & 0x1F) << 11)); // 16 bits for mouseZ

				mouseL = (rxBuffer[i+15] >> 5) & 0x03; // 2 bits for mouseL
				mouseR = (((uint16_t)(rxBuffer[i+15] >> 7) & 0x01)) | ((((uint16_t)rxBuffer[i+16]) & 0x01) << 1); // 2 bits for mouseR
				mouseM = (rxBuffer[i+16] >> 1) & 0x03; // 2 bits for mouseM

				keyboard = ((((uint16_t)rxBuffer[i+16]) >> 3) & 0x1F) | (((uint16_t)rxBuffer[i+17]) << 5) | ((((uint16_t)rxBuffer[i+18]) & 0x07) << 13); // 16 bits for keyboard
				
				// Print everything now!
				printf("ch0 = %u, ch1 = %u, ch2 = %u, ch3 = %u, mode = %u, pause = %u, btnL = %u, btnR = %u, dial = %u, trigger = %u, mouseX = %d, mouseY = %d, mouseZ = %d, mouseL = %u, mouseR = %u, mouseM = %u, keyboard = %u\n", 
					   ch0, ch1, ch2, ch3, mode, pause, btnL, btnR, dial, trigger, mouseX, mouseY, mouseZ, mouseL, mouseR, mouseM, keyboard);

				break; // Exit the loop after 
			}
			// else {
			// 	headerFound = 0;
			// 	printf("No header found!\n");
			// }
				
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