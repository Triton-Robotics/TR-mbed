//
// Created by ankit on 1/30/23.
//
#include <algorithms/PID.h>
#include <motor/DJIMotor.h>
#include <jetson.h>

int main(){

    int i = 0;

    while(true){
        Jetson::set(Jetson::CVDatatype::TeamColor, i);
        Jetson::update();
        i++;
        ThisThread::sleep_for(100ms);
    }
}