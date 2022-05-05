#include "main.hpp"
#include "../util/communications/SerialCommunication.hpp"
DJIRemote myremote(PA_0, PA_1);
CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

//Remote remoteController(A1); 
robotType rType = SENTRY;
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
        //ChassisSubsystem chassis(3,4,0,0,CANHandler::CANBUS_1,M3508);
        Motor chassis1(3,CANHandler::CANBUS_1,M3508);
        Motor chassis2(4,CANHandler::CANBUS_1,M3508);

        //Motor gimbalX(2,CANHandler::CANBUS_1,GM6020);
        Motor gimbalY(6,CANHandler::CANBUS_1,GM6020);
        //gimbalY.setPositionBounds(645, 2383);
        Motor indexer(7,CANHandler::CANBUS_1,M3508);
        while(1){
            myremote.remoteUpdate(); //remoteController.read(); 645 2383
            chassis1.setDesiredSpeed(myremote.getStickData(LEFTJOYX, 0, maxspeed));
            chassis2.setDesiredSpeed(myremote.getStickData(LEFTJOYX, 0, maxspeed));
            gimbalY.setDesiredPos(myremote.getStickData(RIGHTJOYY, 0, 700) + 1500);
            printf("%d\t",(int)(gimbalY.getData(ANGLE)));
            printf("%d\n",(int)(myremote.getStickData(RIGHTJOYY, 0, 700) + 1500));
        }
    }else if(rType == INFANTRY){
        printf("INFANTRY\n");
        ChassisSubsystem chassis(4,2,1,3,CANHandler::CANBUS_1,M3508);
        Motor gimbalX(5,CANHandler::CANBUS_1,GM6020); //NONE OF THESE IDs ARE CORRECT
        Motor gimbalY(6,CANHandler::CANBUS_1,GM6020); //NONE OF THESE IDs ARE CORRECT
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
            chassis.move(
                myremote.getStickData(LEFTJOYY, 0, maxspeed), //x
                myremote.getStickData(LEFTJOYX, 0, maxspeed), //y
                myremote.getStickData(RIGHTJOYX, 0, maxspeed)); //rx
            gimbalY.setDesiredSpeed(myremote.getStickData(RIGHTJOYY, 0, maxspeed));
            gimbalX.setDesiredSpeed(myremote.getStickData(WHEEL, 0, maxspeed));
            printf("%d\n",int(myremote.getStickData(LEFTJOYY, 0, maxspeed)*5));
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

