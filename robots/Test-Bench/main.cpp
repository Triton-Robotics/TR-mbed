//
// Created by ankit on 1/30/23.
//
#include <mbed.h>

DigitalOut led(LED1);

int main(){
    while (true) {
        ThisThread::sleep_for(500);
        led = !led;
        printf("New Serial!\n");
    }
}