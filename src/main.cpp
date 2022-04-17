#include "main.hpp"
#include "../util/communications/SerialCommunication.hpp"

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

Remote remoteController(D15); 
robotType rType = INFANTRY;

//Motor gimly(7,CANHandler::CANBUS_1,GIMBLY);
//Motor m3508(2,CANHandler::CANBUS_1,STANDARD);



int main(){
    Motor::setCANHandler(&canPorts);
    ChassisSubsystem chassis(CANHandler::CANBUS_1,1,2,3,4,M3508);
    while(1){
        
        //printf("%d\n", gimly.getData(ANGLE));
        //gimly.setDesiredCurrent(1000);
        //printf("%d\n", feedback[CANHandler::CANBUS_1][1][2]);

        if(rType == INFANTRY){
            chassis.move(
                remoteController.getChannel(Remote::Channel::LEFT_HORIZONTAL),
                remoteController.getChannel(Remote::Channel::LEFT_VERTICAL),
                remoteController.getChannel(Remote::Channel::RIGHT_HORIZONTAL),3);
            //since max value of output is 660, 1980 seems right for testing purposes.

        }


        Motor::tick();
    }
}

