// #include "mbed.h"
// #include "../src/main.hpp"

// // Either MULTITURNANGLE or VELOCITY or TORQUE
// #define MOTORMODE TORQUE
// #define MOTORTYPE M3508
// #define MOTORID 5

// // Motor IDCanname | MotorType
// CANMotor testMotor(MOTORID, NewCANHandler::CANBUS_1, MOTORTYPE);

// //Thread threadingPrint(osPriorityBelowNormal);

// SerialCommunication Serial(USBTX, USBRX, 9600);
// char mycoolmessage[64];

// // static void printThread(){
// //     while(1)
// //         printf("MULTI:%d\tELAP:%d\tSINCE:%lu\n",testMotor.getData(MULTITURNANGLE), us_ticker_read()/1000,testMotor.timeSinceLastFeedback);
// //     ThisThread::sleep_for(50ms);
// // }

// int main(void) {
//     CANMotor::setCANHandlers(&canHandler1,&canHandler2);
//     //threadingPrint.start(printThread);
//     int val = 0;
    
//     while (1) {
//         if (Serial.PCRead(mycoolmessage)) {
//             val = Serial.toNum(mycoolmessage);
//             if (val != -999) {
//                 if (MOTORMODE == MULTITURNANGLE)
//                     testMotor.setPosition(val);
//                 else if (MOTORMODE == VELOCITY)
//                     testMotor.setSpeed(val);
//                 else if (MOTORMODE == TORQUE)
//                     testMotor.setPower(val);
//                 //printf("VAL:%d\t", val);
//             }
//         }
//         //printf("CUR:%d\tVEL:%d\n",testMotor.getData(TORQUE),testMotor.getData(VELOCITY));
//         //testMotor.printAllMotorData();
//         //printf("MULTI:%d\tELAP:%dSINCE:%lu\n",testMotor.getData(MULTITURNANGLE), us_ticker_read()/1000,testMotor.timeSinceLastFeedback);
//         //printf("Desired: %d Actual: %d \n", val, testMotor.getData(MULTITURNANGLE));
//         ThisThread::sleep_for(1ms); //dunno why but it is VERY IMPORTANT!!
//     }

// }

