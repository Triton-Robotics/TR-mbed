#include "../src/main.hpp"
#include <cstdlib>

// sentry only from 1230 - 1830

CANMotor chassis1(3,NewCANHandler::CANBUS_1,M3508);
CANMotor chassis2(4,NewCANHandler::CANBUS_1,M3508);

PWMMotor leftFlywheelTop(PA_5);
PWMMotor rightFlywheelTop(PA_6);
PWMMotor leftFlywheelBot(PB_6);
PWMMotor rightFlywheelBot(PA_7);

CANMotor yaw(3,NewCANHandler::CANBUS_1,GM6020);
CANMotor pitch(6,NewCANHandler::CANBUS_1,GM6020);

int main()
{
    threadingRemote.start(&remoteThread);
    CANMotor::setCANHandlers(&canHandler1,&canHandler2);

    pitch.setPositionPID(0.0302, 0.0001, 1.9307);
    pitch.setPositionOutputCap(30000);
    pitch.setPositionIntegralCap(10000);
    
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

