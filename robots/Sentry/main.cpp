//
// Created by ankit on 1/30/23.
//
#include <jetson.h>
#include <mbed.h>

#define CLOCK_CYCLE_LENGTH 100

int main(){

    printf("Starting\n");

    DigitalOut led(LED1);
    Jetson::init();

    while(true){

        Jetson::update(CLOCK_CYCLE_LENGTH);
//        Jetson::odom.translation.x += 0.1;
        printf("CV x %f\n", Jetson::cv.vector.x);
        ThisThread::sleep_for(CLOCK_CYCLE_LENGTH);
        led = !led;
    }

    //Jetson::free();
}