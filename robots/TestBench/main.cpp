#include "main.h"

DigitalOut led(L27);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();

    while(true){
        timeStart = us_ticker_read();
        printf("Hey\n");

        if ((timeStart - loopTimer) / 1000 > 25){
            led = !led;

            remoteRead();

            loopTimer = timeStart;
            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}