//
// Created by ankit on 1/30/23.
//
#include <jetson.h>

int main(){

    int i = 0;

    DigitalIn button(BUTTON1);

    Jetson::init();

    while(true){

        if (button){
            i++;
        }

        Jetson::set(Jetson::CVDatatype::TeamColor, i);
    }

    Jetson::free();
}