// #include "mbed.h"
// #include "../src/main.hpp"

// PWMMotor leftFlywheelTop(PA_5);
// PWMMotor rightFlywheelTop(PA_6);
// PWMMotor leftFlywheelBot(PB_6);
// PWMMotor rightFlywheelBot(PA_7);

// // Either MULTITURNANGLE or VELOCITY 

// #define motorID 3
// #define motorType GM6020
// #define MOTORMODE MULTITURNANGLE
// #define outputCap 15000
// #define integralCap 5000
// #define feedForwardVal 0 //useful for overcoming gravity

// PWMMotor LFLYWHEEL(D11);
// PWMMotor RFLYWHEEL(D12);

// int posMotorCommands[] = {4000,5000,6000};
// int speedMotorCommands[] = {0,45*19,90*19};

// CANMotor testMotor(motorID, NewCANHandler::CANBUS_1, motorType);

// float PIDvals[] = {0, 0, 0};

// void updateTestMotorCommand() {
//     if (lS == 1) { 
//             if (MOTORMODE == MULTITURNANGLE)
//                 testMotor.setPosition(posMotorCommands[0]);
//             else 
//                 testMotor.setSpeed(speedMotorCommands[0]);
//     }
//     if (lS == 2) {
//             if (MOTORMODE == MULTITURNANGLE) 
//                 testMotor.setPosition(posMotorCommands[1]);
//             else
//                 testMotor.setSpeed(speedMotorCommands[1]);
//     }
//     if (lS == 3) {
//             if (MOTORMODE == MULTITURNANGLE) 
//                 testMotor.setPosition(posMotorCommands[2]);
//             else
//                 testMotor.setSpeed(speedMotorCommands[2]);
//     }
// }


// void printInfo() {
//         if (rS == 1) {
//             if (MOTORMODE == MULTITURNANGLE) 
//                 printf("Des: %d \t Act: %d", testMotor.getValue(), testMotor.getData(MULTITURNANGLE));
//             if (MOTORMODE == VELOCITY)
//                 printf("Des: %d \t Act: %d", testMotor.getValue(), testMotor.getData(VELOCITY));
//         }
//         else if (rS == 2) {
//             for (int i = 0; i< 3; i++) {
//                 printf("%d", (int)(PIDvals[i]*10000));
//                 //printFloat(PIDvals[i], 3);
//                 printf("\t");
//             }
//             if (MOTORMODE == MULTITURNANGLE) 
//                 printf("Des: %d \t Act: %d\t", testMotor.getValue(), testMotor.getData(MULTITURNANGLE));
//             if (MOTORMODE == VELOCITY)
//                 printf("Des: %d \t Act: %d\t", testMotor.getValue(), testMotor.getData(VELOCITY));
//             //printf("Pwr: %d", testMotor.getPowerOut());
//         }
//         else {
//         }
//         printf("\n");
// }


// int main(void) {
//     CANMotor::setCANHandlers(&canHandler1,&canHandler2);
//     threadingRemote.start(remoteThread);
//     testMotor.pidPosition.feedForward = feedForwardVal;
//     testMotor.justPosError = 1;
    

//     if (outputCap != 0)
//         testMotor.setPositionOutputCap(outputCap);
//     if (integralCap != 0)
//         testMotor.setPositionIntegralCap(integralCap);

//     int count = 0;


//     while (1) {
//         testMotor.pidPosition.setOutputCap(30000);
//         if (MOTORMODE == MULTITURNANGLE)
//             testMotor.setPositionPID(PIDvals[0], PIDvals[1], PIDvals[2]);
//         if (MOTORMODE == VELOCITY)
//             testMotor.setSpeedPID(PIDvals[0], PIDvals[1], PIDvals[2]);
//         ThisThread::sleep_for(1ms);
//         updateTestMotorCommand();
//         ThisThread::sleep_for(1ms);
//         printInfo();

//         PIDvals[0] -= myremote.getStickData(LEFTJOYX, 0, .01);
//         PIDvals[1] -= myremote.getStickData(WHEEL, 0, 0.001);
//         PIDvals[2] -= myremote.getStickData(RIGHTJOYX, 0, .01);

//         for (int i = 0; i < 3; i++) 
//             if (PIDvals[i] < 0)
//                 PIDvals[i] = 0;

//         ThisThread::sleep_for(1ms); //dunno why but it is VERY IMPORTANT!!
//     }
// }
