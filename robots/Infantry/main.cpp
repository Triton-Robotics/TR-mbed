//
// Created by ankit on 1/30/23.
//
#include "mbed.h"
#include <motor/CANMotor.hpp>

int main(){
    DigitalOut led(LED1);

    while(true){
        led = !led;
        printf("Hello, blinky!");
        ThisThread::sleep_for(500ms);
    }
}