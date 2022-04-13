#include "main.hpp"

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

Motor wheel(7,CANHandler::CANBUS_1,GIMBLY);
Motor m3(2,CANHandler::CANBUS_1,STANDARD);

int main(){
    Motor::setCANHandler(&canPorts);
    //m3.setSpeedPID(5, 0, 2);
    while(1){
        //int val = 8738;
        //int8_t bytes[] = {0x2,0x4,0x2,0x4,0x5,0x4,0,0};
        //canPorts.rawSend(0x200, bytes, CANHandler::CANBUS_1);
        //m3.setDesiredSpeed(60);
        printf("%d\n", feedback[CANHandler::CANBUS_1][1][2]);
        Motor::tick();
    }
}

