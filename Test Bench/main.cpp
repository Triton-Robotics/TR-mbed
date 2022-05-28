#include "../src/main.hpp"

// main() runs in its own thread in the OS
CANMotor mots(4,CANHandler::CANBUS_1,M3508);
CANMotor mots1(3,CANHandler::CANBUS_1,M3508);
CANMotor mots2(1,CANHandler::CANBUS_1,M3508);
CANMotor mots3(2,CANHandler::CANBUS_1,M3508);

int main()
{
    CANMotor::setCANHandlers(&canHandler1, &canHandler2);
    int sp = 2000;
    threadingRemote.start(&remoteThread);
    while (true) {
        remotePrint();
        //mots.setPower(sp);
        //mots1.setPower(-sp);
        //mots2.setPower(sp);
        //mots3.setPower(-sp);
        CANMotor::tick();
    }
}

