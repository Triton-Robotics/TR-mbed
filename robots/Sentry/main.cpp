
#include <communications/Jetson.h>
#include <mbed.h>
#include <main.h>

#define CLOCK_CYCLE_LENGTH 25

Jetson jetson(PC_12, PD_2);

Thread cv;

int main(){

//    printf("Starting\n");

//    cv.start([](){
//        while(true){
//            jetson.read();
//        }
//    });

    DigitalOut led(LED1);

    while(true){
        led = !led;
//        printf("Data: %f, %f, %f\n", jetson.get(Jetson::cv::X), jetson.get(Jetson::cv::Y), jetson.get(Jetson::cv::Z));

        double ref_chassis_power = ext_power_heat_data.data.chassis_power;
        printf("Ref power: %i\n", (int) (ref_chassis_power * 100));

        remoteRead();
        printf("RS: %i\n", rS);

        ThisThread::sleep_for(CLOCK_CYCLE_LENGTH);
    }
}