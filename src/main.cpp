#include "main.hpp"

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

Motor wheel(7,CANHandler::CANBUS_1,GIMBLY);
Motor m3(1,CANHandler::CANBUS_1,STANDARD);

int main(){
    Motor::setCANHandler(&canPorts);
    while(1){
        printf("DELETUS\n");
        int val = 8738;
        int8_t bytes[] = {0x2,0x4,0x2,0x4,0x5,0x4,0,0};
        //canPorts.rawSend(0x200, bytes, CANHandler::CANBUS_1);
        m3.setDesiredCurrent(500);
        Motor::tick();
    }
}

