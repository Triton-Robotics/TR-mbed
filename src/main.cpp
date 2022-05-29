// #include "main.hpp"
// #include <ios>
// //CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);
// //NewCANHandler canHandler1(PA_11,PA_12);
// //NewCANHandler canHandler2(PB_12,PB_13);

// Remote remoteController(D9); 
// robotType rType = TEST_BENCH;
// int maxspeed = 300;


// Thread remote(osPriorityHigh);
 
// int main(){
//     printf("Initializing. . .\n");
//     Motor::setCANHandler(&canPorts);
//     remote.start(&remoteThread);
//     printf("Starting robot\n");
//     if(rType == TEST_BENCH){
//         CANMotor::setCANHandlers(&canHandler1,&canHandler2);
//         printf("--TEST_BENCH--\n");
//         // CANMotor test(4,CANHandler::CANBUS_1,M3508);
//         // CANMotor yeet(8,CANHandler::CANBUS_1,M3508);
//         // CANMotor weet(2,CANHandler::CANBUS_1,GM6020);
//         // CANMotor dlete(6,CANHandler::CANBUS_1,GM6020);
//         //ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);

//         CANMotor actualMotor(3,CANHandler::CANBUS_1,M3508);
//         while(1){
//             //remoteController.read(); //myremote.remoteUpdate(); 
//             //printf("%d\n",int(remoteController.getChannel(Remote::Channel::LEFT_HORIZONTAL) * 10000));
//             //test.setPower(700);
//             //dlete.setPower(-100);
//             actualMotor.setPower(500);
//             CANMotor::tick();
//         }
//     }else if(rType == SENTRY){
//         printf("--SENTRY--\n");
//         int gimYBound[2] = {32,96};
//         int gim2YBound[2] = {700,2300};
//         int gimXBound[2] = {-180,180};
//         //ChassisSubsystem chassis(3,4,0,0,CANHandler::CANBUS_1,M3508);
//         Motor chassis1(3,CANHandler::CANBUS_1,M3508);
//         Motor chassis2(4,CANHandler::CANBUS_1,M3508);

//         Motor gimbalX(7,CANHandler::CANBUS_1,GM6020);
//         Motor gimbalY(6,CANHandler::CANBUS_1,GM6020);

//         gimbalY.setPositionBounds(gimYBound[0], gimYBound[1]);
//         gimbalX.setPositionBounds(gimXBound[0], gimXBound[1]);

//         //gimbalY.setPositionPID(20, 0, 0);
//         gimbalY.setPositionIntegralCap(10);
//         gimbalX.setPositionIntegralCap(50);

//         gimbalY.setPositionOutputCap(5000);
//         gimbalY.setSpeedOutputCap(4000);
//         gimbalX.setPositionIntegralCap(2000);

//         PID pidGimY(.1,0,10,0,1000);
//         PID pidGimyPos(7,1,10,2000,4500);

//         PWMMotor leftFlywheelT(PA_5);
//         PWMMotor rightFlywheelT(PA_6);
//         PWMMotor leftFlywheelB(PA_7);
//         PWMMotor rightFlywheelB(PB_6);


//         Motor indexer(2,CANHandler::CANBUS_1,M2006);
//         indexer.setSpeedPID(3,0,2);

//         unsigned long lastTime = us_ticker_read() / 1000;
//         while(1){
            
//             unsigned long Time = us_ticker_read() / 1000;
//             unsigned long timeDifference = Time - lastTime;
//             //myremote.remoteUpdate(); //remoteController.read(); 645 2383

            

//             int gimY = (int)(myremote.getStickData(RIGHTJOYY, 0, (gimYBound[1] - gimYBound[0])/2.0) + gimYBound[0] + (gimYBound[1] - gimYBound[0])/2.0);
//             int gimX = (int)(myremote.getStickData(RIGHTJOYX, 0, 1000));
            
//             int stickRY = int(myremote.getStickData(RIGHTJOYY,0,gim2YBound[1] - gim2YBound[0])) + gim2YBound[0];
//             int gimYAngle = gimbalY.getData(ANGLE);

//             int rY = int(myremote.getStickData(RIGHTJOYY,0,5000));
//             int rX = int(myremote.getStickData(RIGHTJOYX,0,5000));

//             chassis1.setDesiredCurrent(lX);
//             chassis2.setDesiredCurrent(lX);
//             //if(gimY != 1501 && gimY != 637)
//             int calc  = pidGimY.calculate(
//                 stickRY, 
//                 gimYAngle, 
//                 timeDifference);
//             int calcPosOnly = pidGimyPos.calculate(
//                 stickRY,
//                 gimYAngle,
//                 timeDifference);
//             //printf("%d\t%d\t%lu\t%d\n",stickRY,gimYAngle,timeDifference,calcPosOnly);            
//             //gimbalY.setDesiredSpeed(calc);
            
//             gimbalY.setDesiredCurrent(rY);
//             gimbalX.setDesiredCurrent(rX);
//             int indexJamTime = 0;
//             // if(myremote.getSwitchData(LSWITCH) == 2){
//             //     indexer.setDesiredSpeed(0);
//             // }else if(myremote.getSwitchData(LSWITCH) == 1){
//             //     if(abs(indexer.getData(TORQUE)) > 1700){ //jam
//             //         indexJamTime = us_ticker_read() /1000;
//             //     }
//             //     if(us_ticker_read() / 1000 - indexJamTime < 500){
//             //         indexer.setDesiredSpeed(45); //jam
//             //     }else{
//             //         indexer.setDesiredSpeed(-45);
//             //     }
//             // }else if(myremote.getSwitchData(LSWITCH) == 3){
//             //     if(abs(indexer.getData(TORQUE)) > 1700){ //jam
//             //         indexJamTime = us_ticker_read() /1000;
//             //     }
//             //     if(us_ticker_read() / 1000 - indexJamTime < 500){
//             //         indexer.setDesiredSpeed(-45); //jam
//             //     }else{
//             //         indexer.setDesiredSpeed(45);
//             //     }
//             // }
//             indexer.setDesiredCurrent(lY);
//             //printf("rY:%d\n",rY);

//             if(myremote.getSwitchData(RSWITCH) == 2){
//                 leftFlywheelT.set(0);
//                 rightFlywheelT.set(0);
//                 leftFlywheelB.set(0);
//                 rightFlywheelB.set(0);
//             }else if(myremote.getSwitchData(RSWITCH) == 1){
//                 leftFlywheelT.set(80);
//                 rightFlywheelT.set(80);
//                 leftFlywheelB.set(80);
//                 rightFlywheelB.set(80);
//             }else if(myremote.getSwitchData(RSWITCH) == 3){
//                 leftFlywheelT.set(-80);
//                 rightFlywheelT.set(-80);
//                 leftFlywheelB.set(-80);
//                 rightFlywheelB.set(-80);
//             }

//             // for(int j = 0; j < 8;j++){
//             //     printf("%d:%d ",types[0][j],motorOut[0][j]);
//             // }
//             // printf("%d ",gimbalX.getData(ANGLE));
//             // printf("\n");

//             //remotePrint();
//             //printf("lX%d\tlY%d\trX%d\trY%d\tWh%d\n",int(myremote.getStickData(LEFTJOYX,0,1000)),int(myremote.getStickData(LEFTJOYY,0,1000)),int(myremote.getStickData(RIGHTJOYX,0,1000)),int(myremote.getStickData(RIGHTJOYY,0,1000)),int(myremote.getStickData(WHEEL,0,1000)));


//             //printf("%dL  R%d\n",myremote.getSwitchData(LSWITCH),myremote.getSwitchData(RSWITCH));
//             // for(int i = 29; i < 100; i ++){
//             //     gimbalY.setDesiredPos(i);
//             //     ThisThread::sleep_for(10ms);
//             // }
//             // for(int i = 99; i > 28; i --){
//             //     gimbalY.setDesiredPos(i);
//             //     ThisThread::sleep_for(10ms);
//             // }
//             //gimbalX.setDesiredSpeed(gimX);
//             //gimbalX.setDesiredPos(gimX);
//             //printf("X%d\t",(int)(gimbalX.getData(MULTITURNANGLE) * 360.0 / 8191));
//             //printf("DX%d\t",(int)(myremote.getStickData(RIGHTJOYX, 0, gimXBound[1])));
//             //printf("Y%d\t",(int)(gimbalY.getData(MULTITURNANGLE) * 360.0 / 8191));
//             //printf("DY%d\n",(int)(myremote.getStickData(RIGHTJOYY, 0, 10000)));
//             //printf("%d\n",gimY);
//             //ThisThread::sleep_for(10ms);
//             lastTime = Time;
//         }
//     }else if(rType == INFANTRY){
//         printf("--INFANTRY--\n");
//         ChassisSubsystem chassis(4,2,1,3,CANHandler::CANBUS_1,M3508);
//         Motor gimbalX(5,CANHandler::CANBUS_1,GM6020); //NONE OF THESE IDs ARE CORRECT
//         Motor gimbalY(6,CANHandler::CANBUS_1,GM6020); //NONE OF THESE IDs ARE CORRECT
//         while(1){
//             //myremote.remoteUpdate(); //remoteController.read();
//             chassis.move(
//                 myremote.getStickData(LEFTJOYY, 0, maxspeed), //y
//                 myremote.getStickData(LEFTJOYX, 0, maxspeed), //x
//                 myremote.getStickData(RIGHTJOYX, 0, maxspeed)); //rx
//             gimbalY.setDesiredSpeed(myremote.getStickData(RIGHTJOYY, 0, maxspeed));
//             gimbalX.setDesiredSpeed(myremote.getStickData(WHEEL, 0, maxspeed));
//             printf("%d\n",int(myremote.getStickData(LEFTJOYY, 0, maxspeed)*5));
//         }
//     }else if(rType == HERO){
//         printf("--HERO--\n");
//         ChassisSubsystem chassis(4,3,1,2,CANHandler::CANBUS_1,M3508);
//         Motor turretX(5,CANHandler::CANBUS_1,M3508);
//         Motor turretY(6,CANHandler::CANBUS_1,GIMBLY); 
//         Motor indexer(7,CANHandler::CANBUS_1,GIMBLY);
//         Motor duck(8,CANHandler::CANBUS_1,M2006); 
//         PWMMotor flyWheelL(PA_5);
//         PWMMotor flyWheelR(PA_6);
//         while(1){
//             //myremote.remoteUpdate(); //remoteController.read();
//             int dats[7] = {0,0,0,0,0,0,0};
//             myremote.getArray(dats);
//             chassis.move(
//                 lY,
//                 lX,
//                 Wh);

//             //turretY.setDesiredCurrent(lY);
//             //turretX.setDesiredCurrent(lX);

//             // if(myremote.getSwitchData(RSWITCH) == 2){
//             //     duck.setDesiredCurrent(0);
                
//             // }else if(myremote.getSwitchData(RSWITCH) == 1){
//             //     duck.setDesiredCurrent(-1200);
                
//             // }else if(myremote.getSwitchData(RSWITCH) == 3){
//             //     duck.setDesiredCurrent(650);
                
//             // }

//             if(myremote.getSwitchData(LSWITCH) == 2){
//                 flyWheelL.set(0);
//                 flyWheelR.set(0);
//                 //indexer.setDesiredCurrent(0);
//             }else if(myremote.getSwitchData(LSWITCH) == 1){
//                 flyWheelL.set(100);
//                 flyWheelR.set(100);
//                 //indexer.setDesiredCurrent(6000);
//             }else if(myremote.getSwitchData(LSWITCH) == 3){
//                 flyWheelL.set(-100);
//                 flyWheelR.set(-100);
//                 //indexer.setDesiredCurrent(-6000);
//             }

//             indexer.setDesiredCurrent(10000 * (myremote.getSwitchData(LSWITCH) - 2));

//             turretX.setDesiredCurrent(-rX);
//             turretY.setDesiredCurrent(rY * 15);

//             for(int j = 0; j < 8;j++){
//                 printf("%d ",motorOut[0][j]);
//             }
//             printf("\n");

//             // for (int i = 0; i < 7; i++)
//             //     printf("%d\t", dats[i]);
//             // printf("%d\t%d\t%d\t%d\t%d\t%d\t",lX,lY,rX,rY,lS,rS);
//             // printf("\n");

//             // printf("lX%d\tlY%d\tlX%d\tlY%d\tWh%d\n",
//             // int16_t(myremote.getRawStick(LEFTJOYX)),
//             // int16_t(myremote.getRawStick(LEFTJOYY)),
//             // int16_t(myremote.getRawStick(RIGHTJOYX)),
//             // int16_t(myremote.getRawStick(RIGHTJOYY)),
//             // int16_t(myremote.getRawStick(WHEEL)));
//         }
//     }else if(rType == ENGINEER){
//         printf("--ENGINEER--\n");
//         ChassisSubsystem chassis(1,2,3,4,CANHandler::CANBUS_1,M3508);
//         while(1){
//             myremote.remoteUpdate(); //remoteController.read();
//         }
//     }
// }


