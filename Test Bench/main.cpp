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

    while (true) {

        remoteRead();

        if(lS == 2){
            gimbly9.setSpeed(300);
        }else{
            gimbly9.setPower(0);
        }
        CANMotor::sendValues();
        
        CANMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

