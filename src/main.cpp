#include "main.hpp"
#include "../util/communications/SerialCommunication.hpp"

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

Remote remoteController(A1); 
robotType rType = INFANTRY;

//Motor gimly(7,CANHandler::CANBUS_1,GIMBLY);
//Motor m3508(2,CANHandler::CANBUS_1,STANDARD);



int main(){
    printf("idk maybe this will work?\n");
    //Motor::setCANHandler(&canPorts);
    //ChassisSubsystem chassis(CANHandler::CANBUS_1,1,2,3,4,M3508);
    while(1){
        remoteController.read();
        double x = remoteController.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        double y = remoteController.getChannel(Remote::Channel::LEFT_VERTICAL);
        double rx = remoteController.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
        printf("Lx%d\t,Ly%d\t,Rx%d\n",(int)(x * 10000),(int)(y * 10000),(int)(rx * 10000));
        
        // printf("%d\n", int(remoteController.getChannel(Remote::Channel::LEFT_HORIZONTAL)* 100));
        //printf("%d\n", gimly.getData(ANGLE));
        //gimly.setDesiredCurrent(1000);
        //printf("%d\n", feedback[CANHandler::CANBUS_1][1][2]);
        //printf("yettus\n");
        // if(rType == INFANTRY){
            //printf("Dataaa.");
            // double x = remoteController.getChannel(Remote::Channel::LEFT_HORIZONTAL);
            // double y = remoteController.getChannel(Remote::Channel::LEFT_VERTICAL);
            // double rx = remoteController.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
            //chassis.move(x,y,rx,3);
            // printf("Lx%d\t,Ly%d\t,Rx%d\n",(int)(x * 10000),(int)(y * 10000),(int)(rx * 10000));
            //since max value of output is 660, 1980 seems right for testing purposes.

            //chassis.move(1,0,0,60);

        //}


        //Motor::tick();
    }
}

