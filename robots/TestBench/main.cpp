#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include "util/peripherals/imu/ISM330.h"

DigitalOut led(L25);
DigitalOut led2(L26);
DigitalOut led3(L27);
DigitalOut ledbuiltin(LED1);

//CONSTANTS
constexpr float LOWERBOUND = -35.0;
constexpr float UPPERBOUND = 40.0;

//DEGREES PER SECOND AT MAX
constexpr float JOYSTICK_SENSITIVITY_YAW_DPS = 180.0; 
constexpr float JOYSTICK_SENSITIVITY_PITCH_DPS = 180.0;

// Mouse sensitivity initialized
constexpr float MOUSE_SENSITIVITY_YAW_DPS = 10.0;
constexpr float MOUSE_SENSITIVITY_PITCH_DPS = 10.0;

constexpr int OUTER_LOOP_DT_MS = 1;

constexpr int PRINT_FREQUENCY = 8; //the higher the number, the less often

constexpr float CHASSIS_FF_KICK = 0.065;

constexpr float pitch_zero_offset_ticks = 1500;

constexpr int NUM_BALLS_SHOT = 3;
constexpr int FLYWHEEL_VELO = 5500;

#define READ_DEBUG 0
#define MAGICBYTE 0xEE
#define USE_IMU

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);

SPI spiIMU(PB_15, PB_14, PA_9, PB_9); // MOSI, MISO, SCK, NSS/CS
// ISM330 imu2(spiIMU, D10);


int main(){
    spiIMU.format(8, 0);
    spiIMU.frequency(1000000);
    // potentially needed to read from data registers (PG 38 of datasheet)
    spiIMU.write(0x01 | 0x80);
    spiIMU.write(0x40);

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    while (true){
        timeStart = us_ticker_read();
        if (timeStart - loopTimer > OUTER_LOOP_DT_MS) {
            loopTimer = us_ticker_read();
            uint8_t whoami = 0x6B; // 0b0110_1011
            uint8_t whoami2 = spiIMU.write(0x8F);
            uint8_t read_whoami = spiIMU.write(0x00);

            // printff("%x, %x\n", whoami, read_whoami);
            if (whoami == read_whoami) {
                spiIMU.write(0x22 | 0x80);
                uint8_t pitch_x_l = spiIMU.write(0x00);
                
                spiIMU.write(0x23 | 0x80);
                uint8_t pitch_x_h = spiIMU.write(0x00);

                uint16_t pitch_x = (static_cast<uint16_t>(pitch_x_h) << 8) | pitch_x_l;

                printff("pitch: %.2f\n", pitch_x);
            }
        }
        ThisThread::sleep_for(1ms);
    }
}