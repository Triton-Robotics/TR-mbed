//#include "main.h"
//#include "peripherals/oled/Adafruit_SSD1306.h"
////#include "peripherals/oled/oledDisplay.h"
//
//DigitalOut led(L27);
//I2C i2c(PB_7, PB_8);
////Adafruit_SSD1306_I2c oled(i2c, PA_7);
////oledDisplay jingoled = oledDisplay();
//
////DJIMotor wheel1(1,CANHandler::CANBUS_1, M3508, "wheel1");
////DJIMotor wheel2(2,CANHandler::CANBUS_1, M3508, "wheel2");
////DJIMotor wheel3(3,CANHandler::CANBUS_1, M3508, "wheel3");
////DJIMotor wheel4(4,CANHandler::CANBUS_1, M3508, "wheel4");
//
//
////why does the oled screen start writing on random places
//int main(){
//
//    printf("OLED test start\n");
//
////    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
////    DJIMotor::s_sendValues();
////    DJIMotor::s_getFeedback();
//
//    unsigned long timeStart;
//    unsigned long loopTimer = us_ticker_read();
//
//    while(true){
//        timeStart = us_ticker_read();
//
//        if ((timeStart - loopTimer) / 1000 > 15){
//            led = !led;
//
//            remoteRead();
//
//            loopTimer = timeStart;
//            DJIMotor::s_sendValues();
//
//            printff("OLED test start\n");
//
//            oled.writeChar('c');
//            oled.display();
//            //printing how many motor is connected
////            jingoled.motorCount();
//        }
//        DJIMotor::s_getFeedback();
//        ThisThread::sleep_for(5ms);
//    }
//}


/* Scan I2C bus on specified pins and prints out
* the all address where an active device responds.
*
  By Joseph Ellsworth CTO of A2WH
  Take a look at A2WH.com Producing Water from Air using Solar Energy
  March-2016 License: https://developer.mbed.org/handbook/MIT-Licence
  Please contact us http://a2wh.com for help with custom design projects.


* Don't forget 3K pull-up resistors on sda,scl
*
* I tested this by soldering in a I2C chip known to respond at
* address dec=120 hex=70 and the utility got the ack as expected.
* when the chip was de-soldered it was no longer detected.
*/

#include "mbed.h"
#include "peripherals/oled/Adafruit_SSD1306.h"


#define D_SDA                  PB_7
#define D_SCL                  PB_8
// sda=PB7, scl=PB_6 Pins specific to Nucleo-F303K8
// must change pins to match your board.

I2C i2c(D_SDA, D_SCL);
Adafruit_SSD1306_I2c oled(i2c,PA_7);

DigitalOut myled(PC_0);

int ack;
int address;
void scanI2C() {
    for(address=1;address<127;address++) {
        ack = i2c.write(address, "11", 1);
        if (ack == 0) {
            printf("\tFound at %3d -- %3x\r\n", address,address);
        }
        ThisThread::sleep_for(1ms);
    }
}

int main() {
    printf("I2C scanner \r\n");
    scanI2C();
    printf("Finished Scan\r\n");
    // just blink to let us know the CPU is alive
    while(1) {oled.writeChar('c');
        oled.display();
        ThisThread::sleep_for(5ms);
        myled = !myled;

    }
}