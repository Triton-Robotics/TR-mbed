#include "../src/main.hpp"
#include <cstdlib>

// sentry only from 1230 - 1830

CANMotor chassis1(1,NewCANHandler::CANBUS_1,M3508);


int main()
{
    CANMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    
    unsigned long cT;
    while (true) {
        chassis1.setPower(500);

        CANMotor::getFeedback();
        CANMotor::sendValues();
        ThisThread::sleep_for(1ms);
    }
}

