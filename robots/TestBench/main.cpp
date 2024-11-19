#include "main.h"

DigitalOut led(L27);
DigitalOut led2(L26);
DigitalOut led3(L25);
DigitalOut ledbuiltin(LED1);

#define IMPULSE_DT 100
#define IMPULSE_STRENGTH 16383

I2C i2c(PB_7, PB_8);

int main(){

    DJIMotor::s_setCANHandlers(&canHandler1, &canHandler2, false, false);
    DJIMotor::s_sendValues();
    DJIMotor::s_getFeedback();

    unsigned long timeStart;
    unsigned long loopTimer = us_ticker_read();
    int refLoop = 0;

    int motorSpeed = 0;

    while(true){
        timeStart = us_ticker_read();

        // char opr[2] = {0x3F, 0x20};
        char opr[2] = {0x3F, 0x20};
        i2c.write(0x29 << 1, opr, 2, true);
        
        char opr[2] = {0x3B, 0x0C};
        i2c.write(0x29 << 1, opr, 2, true);

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
            
            //i2c

            int len = 32;
            char data[len] = {0};
            data[0] = 0x08;
            i2c.write(0x29 << 1, data, 1, true);
            ThisThread::sleep_for(10ms);
            i2c.read(0x29 << 1, data, len, false);

            //printff("%d %d : %x %x\n",w,r,data[0], data[1]);
            for(int i = 0; i < len; i += 2){
                printff("%x %x %.1f ", data[i], data[i+1], float(data[i] | (data[i + 1] << 8)));
            }
            printff("\n");

            DJIMotor::s_sendValues();
        }
        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}