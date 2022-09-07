// #include "../src/main.hpp"
// #include <cstdlib>

// #define PI 3.14159265

// CANMotor LF(4,NewCANHandler::CANBUS_1,M3508); CANMotor RF(2,NewCANHandler::CANBUS_1,M3508); CANMotor LB(1,NewCANHandler::CANBUS_1,M3508); CANMotor RB(3,NewCANHandler::CANBUS_1,M3508);
// float speedmultiplier = 3;
// float powmultiplier = 2;
// float translationalmultiplier = 3;
// float beybladespeedmult = 1;

// CANMotor yaw(5, NewCANHandler::CANBUS_1, GIMBLY);
// CANMotor pitch(6, NewCANHandler::CANBUS_1, GIMBLY);
// int pitchval = 0;
// #define LOWERBOUND 1000
// #define UPPERBOUND 2000

// CANMotor indexer(7, NewCANHandler::CANBUS_1, C610);
// int indexJamTime = 0;
// int lastJam = 0;

// unsigned long cT = 0;
// unsigned long forwardTime = 250;
// unsigned long reverseTime = 300;
// unsigned long totalTime;

// PWMMotor RFLYWHEEL(D12); PWMMotor LFLYWHEEL(D11);
// PWMMotor flyWheelMotors[] = {RFLYWHEEL, LFLYWHEEL};

// void setFlyWheelPwr(int pwr) {
//     for (int i = 0; i < 2; i++)
//         flyWheelMotors[i].set(pwr);
// }

// int main()
// {
    
//     threadingRemote.start(&remoteThread);
//     //threadingReferee.start(&refereeThread);
//     CANMotor::setCANHandlers(&canHandler1,&canHandler2, true, false);

//     LB.setSpeedPID(1.75, 0.351, 5.63);
//     RF.setSpeedPID(1.073, 0.556, 0);
//     RB.setSpeedPID(1.081, 0.247, 0.386);
//     LF.setSpeedPID(.743, 0.204, 0.284);
//     pitch.setPositionPID(.017,.001,.044);
//     pitch.useAbsEncoder = 1;
//     yaw.setSpeedPID(78.181, 7.303, 1.227);
//     indexer.setSpeedPID(0.014, 0.001, 0.046);
//     indexer.setSpeedIntegralCap(500000);

//     while (true) {

//         LF.outCap = 16000;   
//         RF.outCap = 16000;
//         LB.outCap = 16000;
//         RB.outCap = 16000;
        

//         if(rS == 1){ // All non-serializer motors activated
//             int LFa = lY + lX*translationalmultiplier + rX, RFa = lY - lX*translationalmultiplier - rX, LBa = lY - lX*translationalmultiplier + rX, RBa = lY + lX*translationalmultiplier - rX;
//             // LF.setSpeed(LFa * speedmultiplier);
//             // RF.setSpeed(-RFa* speedmultiplier);
//             // LB.setSpeed(LBa* speedmultiplier);
//             // RB.setSpeed(-RBa* speedmultiplier);
            

//             // if (pitchval < LOWERBOUND) // lowerbound
//             //     pitchval = LOWERBOUND;
//             // if (pitchval > UPPERBOUND) //upperbound
//             //     pitchval = UPPERBOUND;
//             // pitchval += (int)myremote.getStickData(RIGHTJOYY, 0, 3);
//             // pitch.setPosition(pitchval);
//             LF.setPower(LFa * powmultiplier * 1.5);
//             RF.setPower(-RFa * powmultiplier * 1.5);
//             LB.setPower(LBa * powmultiplier);
//             RB.setPower(-RBa * powmultiplier);
            
//             pitch.setPower(rY*9);
//             //yaw.setSpeed(rX/100);
            

//         }else if(rS == 2){ //disable all the non-serializer components
//             LF.setPower(0);RF.setPower(0);LB.setPower(0);RB.setPower(0);
//             yaw.setPower(0); pitch.setPower(0);
//         }else if(rS == 3){ // beyblade mode
//             // double angle = yaw.getData(ANGLE) / 8192.0 * PI * 2;
//             // angle -= PI/4;
            
//             // int beybladespeedmult = 2;

//             // float raw_x = lX * beybladespeedmult;
//             // float raw_y = lY * beybladespeedmult;

//             // float x = (float) (raw_x * cos(angle) - raw_y * sin(angle));
//             // float y = (float) (raw_x * sin(angle) + raw_y * cos(angle));

//             // int beyblade_rotation = 2000;
//             // LF.setSpeed(x + y + beyblade_rotation);
//             // RF.setSpeed(x - y + beyblade_rotation);
//             // LB.setSpeed(-x + y + beyblade_rotation);
//             // RB.setSpeed(-x - y + beyblade_rotation);
//             // yaw.setSpeed(20);
//         }

//         if (lS == 3) {
//             indexer.setPower(-5000);
            
//         }else if(lS == 2){ //disable serializer
//             indexer.setPower(0);
//             setFlyWheelPwr(0);
//         }else if(lS == 1){
//             setFlyWheelPwr(40);
//             cT = us_ticker_read()/1000;
//             totalTime = forwardTime + reverseTime;

//             if (cT % totalTime > 0 && cT % totalTime < forwardTime) {
//                 indexer.setPower(-2000); // shoot
//             }
//             else {
//                 indexer.setPower(800); //unjam
//             }
//                 // if(abs(indexer.getData(TORQUE)) > 500 & abs(indexer.getData(VELOCITY)) < 50){ //intial jam detection
//                 //     if (lastJam == 0) {
//                 //         indexJamTime = us_ticker_read() /1000; // start clock
//                 //         //indexer.setPower(-7000);
//                 //         lastJam = 1;
//                 //         printf("jam detected!\n");
//                 //     }
//                 // }
//                 // else 
//                 //     lastJam = 0;
                
//                 // if(lastJam && us_ticker_read() / 1000 - indexJamTime > 75){ // If jam for more than 75ms then reverse
//                 //     indexer.setPower(10000); 
//                 //     printf("Unjamming...\n");
//                 // }else
//                 //     indexer.setSpeed(-75*36); // No Jam, regular state
//         }

//         ThisThread::sleep_for(1ms);
//     }
// }

