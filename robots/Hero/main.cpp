#include <cstdio>
#include "mbed.h"

DigitalOut led(LED1);
int main(){
    while (true){
        led = !led;
        printf("hello world\n");
        ThisThread::sleep_for(100ms);
    }

}
