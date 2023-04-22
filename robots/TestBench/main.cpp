#include "main.h"
#include <cstdlib>

PID pid(0, 0, 0, 0, 0);

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
    //pid.debugPIDterms = true;

    int refLoop=0;

    unsigned long lastTime = 0;

    double p = 12;
    double i = 0.008;
    double d = 0;

    while (true) {
        
        remoteRead();

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            led = !led;

            // refLoop++;
            // if(refLoop > 15){
            //     refereeThread();
            //     refLoop = 0;
            // }

            refereeThread();

            double scale = 1;

            double ref_chassis_power = ext_power_heat_data.data.chassis_power;

            switch (rS)
            {
            case 1:
                p = p + rY/10000.0;
                break;
            case 2:
                i = i + rY/1000000.0;
                break;
            case 3:
                d = d + rY/10000.0;
                break;
            default:
                break;
            }

            pid.setPID(p, i, d);
            
            int16_t power = lY*5;

            unsigned long time = us_ticker_read() / 1000;

            if (ref_chassis_power > 10) {
                scale = abs(pid.calculate(12, ref_chassis_power, time - lastTime));
                power /= scale;
            }

            lastTime = time;

            if (power < 1500)
                m3058_1.setPower(power);
            else
                m3058_1.setPower(1500);

            printf("kp: %f\t ki: %f\t kd: %f\t scale: %f\t input: %d\t ref: %f\n", 
                    pid.getkP(), pid.getkI(), pid.getkD(), scale, m3058_1.getData(POWEROUT), ref_chassis_power);

            //printf("%d\t %d\t %f\t %f\t %f\t %f\n", m3058_1.getData(POWEROUT), 12, ref_chassis_power, pid.getkP(), pid.getkI(), pid.getkD());

            DJIMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
