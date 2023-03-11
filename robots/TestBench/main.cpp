#include "communications/DJIRemote.h"
#include "motor/DJIMotor.h"
#include "mbed.h"
#include "helperFunctions.hpp"


DigitalOut led(LED1);
Remote remote1(PC_5);


int main(){

    unsigned long loopTimer = us_ticker_read() / 1000;
    led = false;

    while (true) {

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            remote1.read();
            //led = !led;

            remote1.printAxisData();
            //printf("%d\n", remote1.getSwitch(Remote::Switch::LEFT_SWITCH) );

        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        ThisThread::sleep_for(1ms);

    }
}