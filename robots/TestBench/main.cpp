//do SSD 1308, no 1306
#include "main.h"
//#include "peripherals/oled/oledDisplay.h"
#include "peripherals/oled/SSD1308.h"
//#include "peripherals/oled/Adafruit_SSD1306.h"


//Thread jingoledThread(osPriorityLow);


DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
#define D_SDA                  PB_7

#define D_SCL                  PB_8
// sda=PB7, scl=PB_6 Pins specific to Nucleo-F303K8
// must change pins to match your board.

I2C i2c(D_SDA, D_SCL);
SSD1308 oled(&i2c,0x78); //PA_7b or 0x78
//Adafruit_SSD1306_I2c(i2c,PA_7);
//oledDisplay *display = new oledDisplay();

int main(){
    while(true) {
        printf("working?\n");
        oled.printf("hello\n");
        oled.fillDisplay();
        printf("yeah\n");
        ThisThread::sleep_for(1ms);
        led = !led;
    }
//    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
//    DJIMotor::s_sendValues();
//    DJIMotor::s_getFeedback();
//
//    //jingoledThread.start(oledLoop);
//    unsigned long timeStart;
//    unsigned long loopTimer = us_ticker_read();
//    int refLoop = 0;
//
//    while(true){
//        timeStart = us_ticker_read();
//
//        if ((timeStart - loopTimer) / 1000 > 25){
//            led = !led;
//
//            refLoop++;
//            if (refLoop >= 5){
//                refereeThread(&referee);
//                refLoop = 0;
//                led2 = !led2;
//            }
//
//
////            remoteRead();
////            display->motorCount();
//            oled.clearDisplay();
//            oled.writeChar('c');
//
//            loopTimer = timeStart;
//            DJIMotor::s_sendValues();
//        }
//        DJIMotor::s_getFeedback();
//
//        ThisThread::sleep_for(1ms);
//    }
}