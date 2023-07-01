#include "mbed.h"
#include "communications/Jetson.h"

Jetson jetson(PC_12, PD_2);
Thread cv;

Timer t;

int main() {
    t.start();

    cv.start([]() {
        while (true) {
            jetson.write();
            //jetson.read();
        }
    });

    DigitalOut led(LED1);
    while (true) {
        led = !led;
        ThisThread::sleep_for(500);
    }
    return 0;
}
