//
// Created by ankit on 1/30/23.
//
#include <algorithms/pid.hpp>
#include <motor/CANMotor.hpp>
int main(){

    DigitalOut led(LED1);
    while(true){
        led = !led;
        ThisThread::sleep_for(500ms);
    }
}