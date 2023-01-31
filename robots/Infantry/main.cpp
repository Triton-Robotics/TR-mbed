//
// Created by ankit on 1/30/23.
//
#include "mbed.h"

#include "motor/DJIMotor.h"
#include <communications/DJIRemote.h>

int main(){
    //Chassis chassis(1, 2, 3, 4);
    //CANMotor motor(1, NewCANHandler::CANBUS_1, M3508);
    Remote remote(PA_12);
    DigitalOut led(LED1);

    while(true){
        led = !led;
        printf("Hello, blinky!");
        ThisThread::sleep_for(500ms);
    }
}