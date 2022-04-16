#include "main.hpp"
#include "../util/communications/SerialCommunication.hpp"

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

Motor gimly(7,CANHandler::CANBUS_1,GIMBLY);
Motor m3508(2,CANHandler::CANBUS_1,STANDARD);

int main(){
    Motor::setCANHandler(&canPorts);
    //m3.setSpeedPID(5, 0, 2);
    while(1){
        
        printf("%d\n", gimly.getData(ANGLE));
        gimly.setDesiredCurrent(1000);
        //printf("%d\n", feedback[CANHandler::CANBUS_1][1][2]);
        Motor::tick();
    }
}

