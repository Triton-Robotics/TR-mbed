// #include "../src/main.hpp"

// #define maxMotorTemp 50

// CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); CANMotor RF(2,NewCANHandler::CANBUS_1,M3508);
// CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);
// float speedMultplier = 1;

// #define YLOWERBOUND 0 
// #define YUPPERBOUND 1000

// CANMotor turretx(5, NewCANHandler::CANBUS_1, M3508); 
// CANMotor turrety(6, NewCANHandler::CANBUS_1, GM6020);
// CANMotor indexer(7, NewCANHandler::CANBUS_1, GM6020);
// int turretYval = 0;

// int indexJamTime = 0; bool lastJam = 0; int indexUnJamTime = 0;

// // PWMMotor RFLYWHEEL(D12); PWMMotor LFLYWHEEL(D11);
// //PWMMotor FlyWheels[2] = {RFLYWHEEL, LFLYWHEEL};
// PWMMotor LFLYWHEEL(D11);
// CANMotor RFLYWHEEL(6,NewCANHandler::CANBUS_1,M3508);


// CANMotor heroMotors[] = {LF, RF, LB, RB, turretx, turrety, indexer};

// void regularMotorTemp() {
//     for (int i = 0; i < 7; i++) 
//         if (heroMotors[i].getData(TEMPERATURE) > maxMotorTemp)
//             printf("Warning, Motor[%d] is over %d!!!\n", i, maxMotorTemp);
// }

// void setFlyWheelPwr(int val) {
//     LFLYWHEEL.set(val);
//     RFLYWHEEL.setPower(-1000 * val/180);
// }

// void keepturretYinBounds() {
//     if (turretYval < YLOWERBOUND)
//         turretYval = YLOWERBOUND;
//     if (turretYval > YUPPERBOUND)
//         turretYval = YUPPERBOUND;
// }

// int main()
// {
//     threadingRemote.start(&remoteThread);
//     threadingReferee.start(&refereeThread);
//     CANMotor::setCANHandlers(&canHandler1,&canHandler2);

//     while (true) {
//         if (rS == 1){ // Everything non-chassis enable
//             if(abs(turretx.getData(TORQUE)) > 2000 & abs(turretx.getData(VELOCITY)) < 5*19) {
//                 turretx.setPower(0);
//                 printf("turretx motor is stalling!\n");
//             }
//             else
//                 turretx.setPower(rX*4);

//             turretYval += rY/100;
//             keepturretYinBounds();
//             turrety.setPosition(turretYval);
//             printf("RFLYWHEEL VELOCITY:%d\n",RFLYWHEEL.getData(TEMPERATURE));
//         }else if(rS == 3){ // Chassis enable 
//             int LFa = lY + lX + rX, RFa = lY - lX - rX, LBa = lY - lX + rX, RBa = lY + lX - rX;
//             LF.setSpeed(LFa*speedMultplier); RF.setSpeed(-RFa*speedMultplier); LB.setSpeed(LBa*speedMultplier); RB.setSpeed(-RBa*speedMultplier);
//             printf("LF:%d\tRF:%d\tLB:%d\tRB:%d\n",LF.powerOut,RF.powerOut,LB.powerOut,RB.powerOut);
//         }else if(rS == 2){ // Disable robot
//             LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
//             turretx.setPower(0); turrety.setPower(0); indexer.setPower(0);
//             LFLYWHEEL.set(0);
//             RFLYWHEEL.setPower(0);
//             remotePrint();
//         }
        
//         if (rS != 2) {
//             if (lS == 1) {
//                 indexer.setPower(rY * 8);
//                 setFlyWheelPwr(100);
//             }else if(lS == 2){ // Disable serializer
//                 indexer.setPower(0);
//                 setFlyWheelPwr(0);

//             }else if(lS == 3){ // Start serializing with anti-jam code
//                 setFlyWheelPwr(60);

//                 if(abs(indexer.getData(TORQUE)) > 2000 & abs(indexer.getData(VELOCITY)) < 20){ //intial jam detection
//                     if (lastJam == 0) {
//                         indexJamTime = us_ticker_read() /1000; // start clock
//                         lastJam = 1;
//                         printf("jam detected!\n");
//                     }
//                 }
//                 else 
//                     lastJam = 0;
                
//                 if(lastJam && us_ticker_read() / 1000 - indexJamTime > 250){ // If jam for more than 250ms then reverse
//                     indexer.setPower(-7500); 
//                     printf("Jammed\n");
//                 }else
//                     indexer.setSpeed(50); // No Jam, regular state
                
//             }
//         }

//         ThisThread::sleep_for(1ms); 
//         regularMotorTemp();
//     }
// }

// /*
// w = ly + 100;
// s = ly - 100;
// a = rx + 100
// d = rx - 100;
// q = lx + 100;
// e = lx - 100;
// right click (toggle) = rev flywheels
// left click hold = start serializing 
// mousey = gimbaly
// mousex = gimbalx

// */