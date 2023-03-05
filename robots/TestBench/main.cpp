#include <cstdlib>
#include "communications/DJIRemote.h"
#include "motor/DJIMotor.h"
#include "mbed.h"
#include "helperFunctions.hpp"

// #include "o-led/Adafruit_SSD1306.h"
// #include "o-led/Adafruit_SSD1306.cpp"
// #include "o-led/Adafruit_GFX.h"
// #include "o-led/Adafruit_GFX.cpp"
// #include "o-led/Adafruit_GFX_Config.h"

// SPIPreInit gSpi(p5,NC,p7);
// Adafruit_SSD1306_Spi gOled1(gSpi,p26,p25,p24);

//I2C gI2C(PB_9,PB_8);
//Adafruit_SSD1306_I2c gOled2(gI2C,PA_13);

// sentry only from 1230 - 1830
AnalogIn ain(A5);
DigitalOut led(LED1);
DigitalIn button(BUTTON1);

Remote remote1(D9);

int main()
{
    //DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false); // this was missing 1

    int numMotor = 0;
    int stateLS = 0;

    unsigned long loopTimer = us_ticker_read() / 1000;

    //int countLoops = 0;
    int refLoop=0;

    while (true) {

        //printf("Analog V: %f \n",ain.read_voltage()); // testing analog V

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            //printf("*Potato*  %d\n", countLoops);
            led = !led;
            // if(countLoops % 50 == 1)
            //     led = !led;

            remote1.read();
            // WRITE YOUR CODE HERE-------------------------------- expect -600 to 600; -1k to 1k

            //print bits
            /*int printBits [16];
            int16ToBitArray()
            printArray(printBits,16)*/

            //printf("%f\n", remote1.getChannel(Remote::Channel::RIGHT_VERTICAL)); //cast the channel data to solve the bug; mult. by 1000 to amplify.
            //printf("%d\n", remote1.getSwitch(Remote::Switch::LEFT_SWITCH) );
            //DJIMotor::sendValues(); // this was missing 2

        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        //DJIMotor::getFeedback(); // this was missing 3
        ThisThread::sleep_for(1ms);
        //countLoops ++;

    }
}