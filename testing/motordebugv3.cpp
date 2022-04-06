// #include "mbed.h"
// #include "SerialCommunication.hpp"
// #include "motor.hpp"
// #include <cctype>
// #include "CANMsg.h"

// // Canname | MotorType
// Motor testMotor(1, C620);

// SerialCommunication Serial(USBTX, USBRX, 9600);
// char mycoolmessage[64];


// bool isNum(char message[]) {
//     int i = 0;
//     if(message[0] == '-')
//         i = 1;
//     while(message[i] != '\0') {

//         if (!isdigit(message[i]))
//             return false;
//         i++;
//     }
//     return true;
// }

// int main(void) {
//     while (1) {
        
//         if (Serial.update(mycoolmessage) & isNum(mycoolmessage)) {
//             testMotor.setDesiredCurrent(std::atoi(mycoolmessage));
//         }
//         Motor::update();
//     }

// }