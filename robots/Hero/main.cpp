#include <cstdio>
#include "mbed.h"

//
// Created by ankit on 1/30/23.
//
int main(){
    while (true){
        printf("hello world\n");
        ThisThread::sleep_for(100ms);
    }

}
