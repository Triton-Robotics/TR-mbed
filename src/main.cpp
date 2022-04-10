#include "main.hpp"

Motor wheel(7,GIMBLY);
Motor m3(1,STANDARD);
CANHandler nucleoCan(PA_11,PA_12,PB_12,PB_13);

int main(){
    motorDebug = 1;
    while(1){
        printf("DELETUS\n");
        int val = 8738;
        int bytes[] = {0,0,0,0,0x01,0xF4,0,0};
        nucleoCan.rawSend(0x200, bytes, CANHandler::CANBUS_1);
    }
}