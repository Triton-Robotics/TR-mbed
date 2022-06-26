// #include "../src/main.hpp"
// #include <cstdlib>

// #define PI 3.14159265

// bool eStop = 0;

// int kY, kX, kR;
// CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); 
// CANMotor RF(2,NewCANHandler::CANBUS_1,M3508); 
// CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); 
// CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);
// int baseSpeed = 90*19;

// float strafemultiplier = 2;
// float beybladespeedmult = 1;

// CANMotor yaw(5, NewCANHandler::CANBUS_1, GIMBLY);
// CANMotor pitch(6, NewCANHandler::CANBUS_1, GIMBLY);
// int pitchval = 0;
// int yawval = 0;
// #define LOWERBOUND 1000
// #define UPPERBOUND 2000

// CANMotor indexer(7, NewCANHandler::CANBUS_1, C610);
// int indexJamTime = 0;
// int lastJam = 0;

// PWMMotor RFLYWHEEL(D12); PWMMotor LFLYWHEEL(D11);
// PWMMotor flyWheelMotors[] = {RFLYWHEEL, LFLYWHEEL};

// int lastTick;
// int deltaTick;


// void setFlyWheelPwr(int pwr) {
//     for (int i = 0; i < 2; i++)
//         flyWheelMotors[i].set(pwr);
// }

// int main()
// {
//     threadingRemote.start(&remoteThread);
//     threadingReferee.start(&refereeThread);
//     CANMotor::setCANHandlers(&canHandler1,&canHandler2);

//     LB.setSpeedPID(1.75, 0.351, 5.63);
//     RF.setSpeedPID(1.073, 0.556, 0);
//     RB.setSpeedPID(1.081, 0.247, 0.386);
//     LF.setSpeedPID(.743, 0.204, 0.284);
//     pitch.setPositionPID(.017,.001,.044);
//     pitch.useAbsEncoder = 1;
//     yaw.setSpeedPID(78.181, 7.303, 1.227);
//     indexer.setSpeedPID(0.014, 0.001, 0.046);

//     while (true) {

//         //printf("level: %d\n", ext_game_robot_state.data.robot_level);
//         deltaTick = us_ticker_read()/1000 - lastTick;
//         //if (deltaTick > 10) {
//             printf("time since last tick: %d\n", deltaTick);
//         //}
        
//         lastTick = us_ticker_read()/1000;  

//         LF.outCap = 16000;   
//         RF.outCap = 16000;
//         LB.outCap = 16000;
//         RB.outCap = 16000;

//         eStop = myremote.getToggleState(CTRL);
        
//         // if (!eStop) {
//             int LFa = kY + kX*strafemultiplier + kR, RFa = kY - kX*strafemultiplier - kR, LBa = kY - kX*strafemultiplier + kR, RBa = kY + kX*strafemultiplier - kR;

//             if (!myremote.getKeyState(SHIFT)) { 
//                 kY = 0; kX = 0; kR = 0;

//                 if (myremote.getKeyState(W)) 
//                     kY += baseSpeed;
//                 if (myremote.getKeyState(S))
//                     kY -= baseSpeed;
//                 if (myremote.getKeyState(A))
//                     kR -= baseSpeed;
//                 if (myremote.getKeyState(D))
//                     kR += baseSpeed;   
//                 if (myremote.getKeyState(E))
//                     kX += baseSpeed;  
//                 if (myremote.getKeyState(Q))
//                     kX -= baseSpeed;  
//                 LF.setSpeed(LFa);
//                 RF.setSpeed(-RFa);
//                 LB.setSpeed(LBa);
//                 RB.setSpeed(-RBa);
//             }
//             else { // beyblade mode
//                 double angle = yaw.getData(ANGLE) / 8192.0 * PI * 2;
//                 angle -= PI/4;
                
//                 int beybladespeedmult = 2;

//                 float raw_x = lX * beybladespeedmult;
//                 float raw_y = lY * beybladespeedmult;

//                 float x = (float) (raw_x * cos(angle) - raw_y * sin(angle));
//                 float y = (float) (raw_x * sin(angle) + raw_y * cos(angle));

//                 int beyblade_rotation = 2000;
//                 LF.setSpeed(x + y + beyblade_rotation);
//                 RF.setSpeed(x - y + beyblade_rotation);
//                 LB.setSpeed(-x + y + beyblade_rotation);
//                 RB.setSpeed(-x - y + beyblade_rotation);
//                 yaw.setSpeed(20);
//             }
//             ThisThread::sleep_for(1ms);

//             if (pitchval < LOWERBOUND) 
//                 pitchval = LOWERBOUND;
//             if (pitchval > UPPERBOUND) 
//                 pitchval = UPPERBOUND;
//             pitchval += -(myremote.getMouseData(SPEEDY));
//             pitch.setPosition(pitchval);
//             yaw.setSpeed(-myremote.getMouseData(SPEEDX)*2);

//             ThisThread::sleep_for(1ms);
            
//             if (myremote.getMouseButtonTogState(RCLICK)) {
//                 if (myremote.getToggleState(SHIFT))
//                     setFlyWheelPwr(30); // Toggle between lvl 0 firing speed 
//                 else
//                     setFlyWheelPwr(60);
//                 if (myremote.getMouseData(LCLICK)) {
//                     if(abs(indexer.getData(TORQUE)) > 500 & abs(indexer.getData(VELOCITY)) < 20){ //intial jam detection
//                         if (lastJam == 0) {
//                             indexJamTime = us_ticker_read() /1000; // start clock
//                             lastJam = 1;
//                             printf("jam detected!\n");
//                         }
//                     }
//                     else 
//                         lastJam = 0;
                    
//                     if(lastJam && us_ticker_read() / 1000 - indexJamTime > 75){ // If jam for more than 75ms then reverse
//                         indexer.setPower(7500); 
//                         printf("Unjamming...\n");
//                     }else
//                         indexer.setSpeed(-50*36); // No Jam, regular state
//                 }
//                 else 
//                     indexer.setPower(0);
//             }
//             else {
//                 setFlyWheelPwr(0);
//                 indexer.setPower(0);
//             }


//         // }
//         // else { //Disable robot
//         //     printf("Robot disabled..\n");
//         //     LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
//         //     yaw.setPower(0); pitch.setPower(0);
//         //     setFlyWheelPwr(0);
//         // }

//         ThisThread::sleep_for(1ms);      
        
//     }
// }

