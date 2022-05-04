#include "main.hpp"
#include "../util/communications/SerialCommunication.hpp"

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);
DJIRemote myremote(PA_0, PA_1);
Remote remoteController(A1); 
robotType rType = INFANTRY;
int maxspeed = 300;

int main(){
    printf("Starting robot\n");
    Motor::setCANHandler(&canPorts);
    if(rType == TEST_BENCH){
        //ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
            
        }
    }else if(rType == SENTRY){
        ChassisSubsystem chassis(3,4,0,0,CANHandler::CANBUS_1,M3508);
        Motor gimbalX(2,CANHandler::CANBUS_1,GM6020);
        Motor gimbalY(5,CANHandler::CANBUS_1,GM6020);
        Motor indexer(7,CANHandler::CANBUS_1,M3508);
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
            
        }
    }else if(rType == INFANTRY){
        ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);
        //Motor gimbalX(5,CANHandler::CANBUS_1,GM6020); //NONE OF THESE IDs ARE CORRECT
        //Motor gimbalY(6,CANHandler::CANBUS_1,GM6020); //NONE OF THESE IDs ARE CORRECT
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
            chassis.move(
                myremote.getStickData(LEFTJOYY, 0, maxspeed), //x
                myremote.getStickData(LEFTJOYX, 0, maxspeed), //y
                myremote.getStickData(RIGHTJOYX, 0, maxspeed)); //rx
        }
    }else if(rType == HERO){
        ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);
        //Motor gimbalX(5,CANHandler::CANBUS_1,M3508); //NONE OF THESE IDs ARE CORRECT
        //Motor gimbalY(6,CANHandler::CANBUS_1,GM6020); //NONE OF THESE IDs ARE CORRECT
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
            
        }
    }else if(rType == ENGINEER){
        ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
            
        }
    }
}

