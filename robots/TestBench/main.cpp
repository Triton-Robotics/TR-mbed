#include "main.h"
#include <cstdlib>

PID pid(1, 0, 1, 0, 0);

DJIMotor m3058_1(1, CANHandler::CANBUS_1, M3508);
DJIMotor m3058_2(2, CANHandler::CANBUS_1, M3508);
DJIMotor m3058_3(3, CANHandler::CANBUS_1, M3508);
DJIMotor m3058_4(4, CANHandler::CANBUS_1, M3508);

DigitalOut led(LED1);

int main()
{
    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    
    unsigned long loopTimer = us_ticker_read() / 1000;

    DJIMotor::getFeedback();
    double beybladeSpeed = 2;
    bool beybladeIncreasing = true;

    //pid.debug = true;
    pid.debugPIDterms = true;

    int refLoop=0;

    unsigned long lastTime = 0;

    while (true) {
        
        remoteRead();

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            led = !led;

            refLoop++;
            if(refLoop > 1){
                refereeThread();
                refLoop = 0;
            }

            double scale = 1;

            double pos_scale = 1;

            unsigned long time = us_ticker_read() / 1000;

            double ref_chassis_power = ext_power_heat_data.data.chassis_power;

            scale = pid.calculate(12, ref_chassis_power, time - lastTime);

            lastTime = time;

            double convert;

            if (scale < 0) {
                // convert = scale * (20/16384);
                // m3058_1.setPower((int16_t) (lY*5 + convert));
                pos_scale = abs(scale);
                m3058_1.setPower((int16_t) (lY*5 / pos_scale));
            }
            else {
                m3058_1.setPower((int16_t) (lY*5));
            }

            printf("scale: %f\t pos_scale: %f\t input: %d\t ref: %f\n", 
                    scale, pos_scale, m3058_1.getData(POWEROUT), ref_chassis_power);

            DJIMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
