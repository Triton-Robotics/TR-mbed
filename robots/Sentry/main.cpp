//
// Created by ankit on 1/30/23.
//
#include <jetson.h>
#include <mbed.h>

int main(){

    printf("Starting\n");

    DigitalOut led(LED1);
    Jetson::init();

    while(true){

        Jetson::update(100);
        Jetson::odom.translation.x += 0.1;
        printf("CV x %f\n", Jetson::cv.vector.x);
        ThisThread::sleep_for(100ms);
        led = !led;
    }

    //Jetson::free();
}