//
// Created by ankit on 1/30/23.
//
#include <algorithms/PID.h>
#include <motor/DJIMotor.h>
int main(){

    DigitalOut led(LED1);
    while(true){
        led = !led;
        ThisThread::sleep_for(500ms);
    }
}