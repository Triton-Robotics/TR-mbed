#include "main.hpp"

Motor wheel(7,GIMBLY);
Motor m3(2,STANDARD);
CANHandler nucleoCan(PA_11,PA_12,PB_12,PB_13);

int main(){
    motorDebug = 1;
    while(1){
        printf("DELETUS\n");
        int val = 8738;
        int8_t bytes[] = {0,0,0x2,0x4,0x5,0x4,0,0};
        nucleoCan.rawSend(0x200, bytes, CANHandler::CANBUS_1);
    }
}

