#include "main.h"
#include <cstdlib>

//PID pid(0, 0, 0, 0, 0);

// DJIMotor m3058_1(1, CANHandler::CANBUS_1, M3508);
// DJIMotor m3058_2(2, CANHandler::CANBUS_1, M3508);
// DJIMotor m3058_3(3, CANHandler::CANBUS_1, M3508);
// DJIMotor m3058_4(4, CANHandler::CANBUS_1, M3508);
DJIMotor jaspdexer(7, CANHandler::CANBUS_1, GIMBLY);

DigitalOut led(LED1);

int main()
{
    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    
    unsigned long loopTimer = us_ticker_read() / 1000;

    DJIMotor::getFeedback();

    unsigned long lastTime = 0;

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

            printf("lY %d lS %d dex %d\n",lY, lS, jaspdexer.getData(ANGLE));

            if(lS == 1)
                jaspdexer.setSpeed(lY/20);
            if(lS == 2)
                jaspdexer.setPower(0);
            if(lS == 3)
                jaspdexer.setPower(lY * 5);

            DJIMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}
