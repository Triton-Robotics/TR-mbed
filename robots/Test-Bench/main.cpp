#include "main.h"
#include <cstdlib>

//--------------CLASS TESTER

 DJIMotor m3508_1(1,CANHandler::CANBUS_1,M3508);
 DigitalOut led(LED1); 
 TRMutex tester;

int main() {  
    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    unsigned long loopTimer = us_ticker_read() / 1000;
    int countLoops = 0;
    int refLoop=0;

    while (true) {
        //MUTEX.lock();

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            led = !led;
            remoteRead();

            m3508_1.setPower(3000);
            DJIMotor::sendValues();
            tester.printMutex("hi");
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1);
        countLoops ++;
        //MUTEX.unlock();
        
    }
}





//---------------MUTEX TESTER

// #include "main.h"
// #include <cstdlib>

// void printCode();

// Mutex MUTEX; 
// Thread Thread_print_code;

// DJIMotor m3508_1(5,CANHandler::CANBUS_1,M3508);
// DJIMotor m3508_6(6,CANHandler::CANBUS_1,M3508);
// DigitalOut led(LED1);  

// void printCode(){
//     while(1){
//         MUTEX.lock();
//         printf("%d\n",m3508_1.getData(TORQUE));
//         ThisThread::sleep_for(1);
//         MUTEX.unlock();
//     }
// }

// int main() 
// {  
//     DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);

//     Thread_print_code.start(printCode); 
//     unsigned long loopTimer = us_ticker_read() / 1000;
//     int countLoops = 0;
//     int refLoop=0;

//     while (true) {
//         //MUTEX.lock();
//         unsigned long timeStart = us_ticker_read() / 1000;
//         if(timeStart - loopTimer > 25){
//             loopTimer = timeStart;

//             led = !led;
//             remoteRead();

//             m3508_1.setPower(3000);
//             m3508_6.setPower(3000);
//             DJIMotor::sendValues();
//         }
//         unsigned long timeEnd = us_ticker_read() / 1000;
//         DJIMotor::getFeedback();
//         ThisThread::sleep_for(1);
//         countLoops ++;
//         //MUTEX.unlock();
        
//     }
// }


