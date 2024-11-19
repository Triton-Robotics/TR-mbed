#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    bool prevL = false;
    bool switL = false;

    int motorSpeed = 0;

    while(true){
        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > 25){
            loopTimer = timeStart;
            led = !led;
            ledbuiltin = !ledbuiltin;

            refLoop++;
            if (refLoop >= 5){
                refereeThread(&referee);
                refLoop = 0;
                led2 = !led2;
            }
            remoteRead();
            
            

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}