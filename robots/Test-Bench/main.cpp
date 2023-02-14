#include "main.h"
#include <cstdlib>

void printCode();

Mutex MUTEX; 
Thread Thread_print_code;

DJIMotor m3508_1(1,CANHandler::CANBUS_1,M3508);
DigitalOut led(LED1);  

void printCode(){
    while(1){
        MUTEX.lock();
        printf("hi \n");
        ThisThread::sleep_for(1s);
        MUTEX.unlock();
    }
}

int main() 
{   
    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);

    Thread_print_code.start(printCode); 
    unsigned long loopTimer = us_ticker_read() / 1000;
    int countLoops = 0;
    int refLoop=0;

    while (true) {
        MUTEX.lock();
        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            led = !led;
            remoteRead();
            m3508_1.setPower(1000);
            DJIMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1s);
        countLoops ++;
        MUTEX.unlock();
    }
}