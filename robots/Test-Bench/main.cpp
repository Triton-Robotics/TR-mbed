#include "main.h"
#include <cstdlib>
#include "mbed.h"
#include <queue>
// //--------------TIMER
// #include <chrono>
// #include <thread>
// #include <iostream>

// class Time
// {
//     // make things readable
//     using clk = std::chrono::steady_clock;

//     clk::time_point b; // begin
//     clk::time_point e; // end

// public:
//     void clear() { b = e = clk::now(); }
//     void start() { b = clk::now(); }
//     void stop() { e = clk::now(); }

//     friend std::ostream& operator<<(std::ostream& o, const Time& timer)
//     {
//         return o << timer.secs();
//     }

//     // return time difference in seconds
//     double secs() const
//     {
//         if(e <= b)
//             return 0.0;
//         auto d = std::chrono::duration_cast<std::chrono::microseconds>(e - b);
//         return d.count() / 1000000.0;
//     }
// };

//--------------CLASS TESTER

 DJIMotor m3508_1(1,CANHandler::CANBUS_1,M3508);
 DigitalOut led(LED1); 
 TRMutex tester;

int main() {
    //Time timer;
    //int counter =0;

    //timer.start();
    DJIMotor::setCANHandlers(&canHandler1,&canHandler2, false, false);
    unsigned long loopTimer = us_ticker_read() / 1000;
    int countLoops = 0;
    int refLoop=0;

    while (true) {

        unsigned long timeStart = us_ticker_read() / 1000;
        if(timeStart - loopTimer > 25){
            loopTimer = timeStart;

            led = !led;
            remoteRead();

            m3508_1.setPower(1000);
            DJIMotor::sendValues();
            tester.print(m3508_1.getData(TORQUE));
            tester.println("hi");
            tester.printff("ya[%d]\n", 5);
        }
        unsigned long timeEnd = us_ticker_read() / 1000;
        DJIMotor::getFeedback();
        ThisThread::sleep_for(1ms);
        countLoops ++;
        //counter++;

        // if (counter == 10) {
        //     timer.stop();
        //     printf("%d", timer);
        // }
    }
};

// TRMutex::TRMutex() {
//     print_code_thread.start(loop);
    
// }


// string TRMutex::printMutex(string statement) {
//     for (int i = 0; i < statement.length(); i++) {
//         buffer.push(statement[i]);
//     }
    
// }

// void TRMutex::loop() {
//     while (true) {
//         MUTEX.lock();

//         if (!buffer.empty()) {
//             printf("%c", buffer.front());
//             buffer.pop();
//         }
        
//         ThisThread::sleep_for(1ms);
//         MUTEX.unlock();
//     }
// }





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


