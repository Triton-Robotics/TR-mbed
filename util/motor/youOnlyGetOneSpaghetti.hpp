// #include "mbed.h"
// //#include "momsSpaghetti.hpp"
// #include "../communications/causeSpaghettiComesOnceInALifetime.hpp"

// // #pragma once
// // namespace{
// //     class CANMotor{
// //         public:
// //             void tick();
// //     };
// // }

// class CANMotor;

// #ifndef motorhandler_hpp
// #define motorhandler_hpp

// enum sendID{
//     x200 = 0,
//     x1ff = 1,
//     x1FF = 1,
//     x2ff = 2,
//     x2FF = 2
// };

// class MotorHandler{
//     private:
//         enum CANBus {CANBUS_1, CANBUS_2, NOBUS};
//     public:
//         NewCANHandler* canHandler1;
//         NewCANHandler* canHandler2;

//         //CANMotor* motors[4][4]; //motors[0] and [1] are for standards, [2] and [3] are for gimblies

//         int16_t outputs[3][4];

//         MotorHandler(PinName can1Rx, PinName can1Tx, PinName can2Rx, PinName can2Tx)
//         {
//             canHandler1 = new NewCANHandler(can1Rx,can1Tx);
//             canHandler2 = new NewCANHandler(can2Rx,can2Tx);
//         }

//         void send(CANBus bus){
//             bool doSend[3];
//             for(int i = 0; i < 3; i ++){
//                 bool send = false;
//                 for(int j = 0; j < 4; j ++){
//                     if(motors[i][j]->type != NONE){
//                         send = true;
//                     }
//                 }
//                 doSend[i] = send;
//             }

//             for(int j = 0; j < 3; j ++){
//                 if(doSend[j]){

//                     int8_t sentBytes[8] = {0,0,0,0,0,0,0,0};
//                     for(int i = 0; i < 4;  i ++){
//                         sentBytes[(2*i)+1] = motors[j][i]->powerOut & (0xFF);
//                         sentBytes[2*i] = (motors[j][i]->powerOut >> 8) & (0xFF);
//                     }
//                     canHandler1->rawSend(sendIDs[j],sentBytes);
//                 }
//             }
//         }

//         bool validate(){
//             for(int i = 0; i < 4; i ++){
//                 if(motors[1][i]->type != NONE && motors[2][i]->type != NONE){
//                     return false;
//                 }
//             }
//             return true;
//         }



//         void tick(){

//         }
// };

// #endif