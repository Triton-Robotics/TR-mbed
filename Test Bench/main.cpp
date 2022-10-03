#include "../src/main.hpp"
#include <cstdlib>

// sentry only from 1230 - 1830

CANMotor m3508_1(1,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_2(2,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_3(3,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_4(4,NewCANHandler::CANBUS_1,M3508);
CANMotor m3508_5(5,NewCANHandler::CANBUS_1,M3508);
CANMotor gimbly8(4,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly9(5,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly10(6,NewCANHandler::CANBUS_1,GIMBLY);
CANMotor gimbly11(7,NewCANHandler::CANBUS_1,GIMBLY);


int main()
{
    CANMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    
    unsigned long loopTimer;
    while (true) {

        remoteRead();

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 10){
            if(lS == 2){
                m3508_1.setSpeed(500);
                m3508_2.setSpeed(500);
                m3508_3.setSpeed(500);
                m3508_4.setSpeed(500);
                m3508_5.setSpeed(500);
                gimbly8.setSpeed(500);
                gimbly9.setSpeed(500);
                gimbly10.setSpeed(500);
                gimbly11.setSpeed(500);
            }else{
                m3508_1.setPower(0);
                m3508_2.setPower(0);
                m3508_3.setPower(0);
                m3508_4.setPower(0);
                m3508_5.setPower(0);
                gimbly8.setPower(0);
                gimbly9.setPower(0);
                gimbly10.setPower(0);
                gimbly11.setPower(0);
            }
            CANMotor::sendValues();
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        CANMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

