#include "../src/main.hpp"
#include <cstdlib>

CANMotor chassis1(3,NewCANHandler::CANBUS_1,M3508);
CANMotor chassis2(4,NewCANHandler::CANBUS_1,M3508);

PWMMotor leftFlywheelTop(PA_5);
PWMMotor rightFlywheelTop(PA_6);
PWMMotor leftFlywheelBot(PB_6);
PWMMotor rightFlywheelBot(PA_7);

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);
    
    unsigned long cT;
    while (true) {
        cT = us_ticker_read()/1000;
        
        if (cT % 6000 > 0 && cT % 6000 < 3000){
            chassis1.setPower(-1000);
            chassis2.setPower(-1000); // constant move right
        }
        else {
            chassis1.setPower(1000);
            chassis2.setPower(1000);
        }

            
        ThisThread::sleep_for(1ms);
    }
}

