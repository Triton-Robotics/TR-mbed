
#include <communications/Jetson.h>
#include <mbed.h>

#define CLOCK_CYCLE_LENGTH 500

Jetson jetson(PC_12, PD_2);

Thread cv;

int main(){

    printf("Starting\n");

    cv.start([](){
        while(true){
            jetson.read();
        }
    });

    DigitalOut led(LED1);

    while(true){
        led = !led;
        printf("Data: %f, %f, %f\n", jetson.get(Jetson::cv::X), jetson.get(Jetson::cv::Y), jetson.get(Jetson::cv::Z));
        ThisThread::sleep_for(CLOCK_CYCLE_LENGTH);
    }
}