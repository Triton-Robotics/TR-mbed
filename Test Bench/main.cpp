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

CANMotor getMotor(int i){
    switch (i) {
    
    case 1:
        return m3508_1;
        break;
    case 2:
        return m3508_2;
        break;
    case 3:
        return m3508_3;
        break;
    case 4:
        return m3508_4;
        break;
    case 5:
        return m3508_5;
        break;
    case 6:
        return m3508_1;
        break;
    case 7:
        return m3508_1;
        break;
    case 8:
        return gimbly8;
        break;
    case 9:
        return gimbly9;
        break;
    case 10:
        return gimbly10;
        break;
    case 11:
        return gimbly11;
        break;
    default:
        return m3508_1;
        break;
    }
}

int main()
{
    CANMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);

    int numMotor = 0;
    int stateLS = 0;
    while (true) {

        remoteRead();

        if(lS == 2){
            gimbly9.setSpeed(300);
        }else{
            gimbly9.setPower(0);
        }

        gimbly9>>VELOCITY;

        m3508_1 = 100;

        CANMotor::sendValues();
        
        CANMotor::getFeedback();
        ThisThread::sleep_for(1ms);
    }
}

