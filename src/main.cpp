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
        printf("--TEST_BENCH--\n");
        //ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
            
        }
    }else if(rType == SENTRY){
        printf("--SENTRY--\n");
        int gimYBound[2] = {32,96};
        int gimXBound[2] = {-180,180};
        //ChassisSubsystem chassis(3,4,0,0,CANHandler::CANBUS_1,M3508);
        Motor chassis1(3,CANHandler::CANBUS_1,M3508);
        Motor chassis2(4,CANHandler::CANBUS_1,M3508);

        Motor gimbalX(7,CANHandler::CANBUS_1,GM6020);
        Motor gimbalY(6,CANHandler::CANBUS_1,GM6020);

        gimbalY.setPositionBounds(gimYBound[0], gimYBound[1]);
        gimbalX.setPositionBounds(gimXBound[0], gimXBound[1]);

        //gimbalY.setPositionPID(20, 0, 0);
        gimbalY.setPositionIntegralCap(10);
        gimbalX.setPositionIntegralCap(50);

        gimbalY.setPositionOutputCap(5000);
        gimbalX.setPositionIntegralCap(2000);

        PID pidGimY(1,0,10,0,5000);

        Motor indexer(2,CANHandler::CANBUS_1,M3508);
        unsigned long lastTime = us_ticker_read() / 1000;
        while(1){
            
            unsigned long Time = us_ticker_read() / 1000;
            unsigned long timeDifference = Time - lastTime;
            myremote.remoteUpdate(); //remoteController.read(); 645 2383

            int gimY = (int)(myremote.getStickData(RIGHTJOYY, 0, (gimYBound[1] - gimYBound[0])/2.0) + gimYBound[0] + (gimYBound[1] - gimYBound[0])/2.0);
            int gimX = (int)(myremote.getStickData(RIGHTJOYX, 0, 1000));
            
            //chassis1.setDesiredSpeed(myremote.getStickData(LEFTJOYX, 0, maxspeed));
            //chassis2.setDesiredSpeed(myremote.getStickData(LEFTJOYX, 0, maxspeed));
            //if(gimY != 1501 && gimY != 637)
            int calc  = pidGimY.calculate(
                    myremote.getStickData(RIGHTJOYY,0,8191), 
                    gimbalY.getData(ANGLE), 
                    timeDifference);
            printf("%d\t%d\t%lu\t%d\n",int(myremote.getStickData(RIGHTJOYY,0,8191)),gimbalY.getData(ANGLE),timeDifference,calc);            
            gimbalY.setDesiredSpeed(calc);
            // for(int i = 29; i < 100; i ++){
            //     gimbalY.setDesiredPos(i);
            //     ThisThread::sleep_for(10ms);
            // }
            // for(int i = 99; i > 28; i --){
            //     gimbalY.setDesiredPos(i);
            //     ThisThread::sleep_for(10ms);
            // }
            //gimbalX.setDesiredSpeed(gimX);
            //gimbalX.setDesiredPos(gimX);
            //printf("X%d\t",(int)(gimbalX.getData(MULTITURNANGLE) * 360.0 / 8191));
            //printf("DX%d\t",(int)(myremote.getStickData(RIGHTJOYX, 0, gimXBound[1])));
            //printf("Y%d\t",(int)(gimbalY.getData(MULTITURNANGLE) * 360.0 / 8191));
            //printf("DY%d\n",(int)(myremote.getStickData(RIGHTJOYY, 0, 10000)));
            //printf("%d\n",gimY);
            //ThisThread::sleep_for(10ms);
            lastTime = Time;
        }
    }else if(rType == INFANTRY){
        printf("--INFANTRY--\n");
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
        printf("--HERO--\n");
        ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);
        //Motor gimbalX(5,CANHandler::CANBUS_1,M3508); //NONE OF THESE IDs ARE CORRECT
        //Motor gimbalY(6,CANHandler::CANBUS_1,GM6020); //NONE OF THESE IDs ARE CORRECT
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
            
        }
    }else if(rType == ENGINEER){
        printf("--ENGINEER--\n");
        ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);
        while(1){
            myremote.remoteUpdate(); //remoteController.read();
        }
    }
}

