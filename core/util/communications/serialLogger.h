#include "mbed.h"
#include <string>
#include <cstdlib>

#define MAX_PRINT_CHARS 200 //Number of can handlers
#ifndef seriallogger_hpp
#define seriallogger_hpp

using namespace std;

class Logger{
    private:
        int index = 0;
        string toSend = "";
        int delay_ms = 10;

    public:
        void setDelay(int ms) {delay_ms = ms;}
        int getDelay() {return delay_ms;}
        
        void log(string item, int value){
            string tempSend;
            toSend = toSend + item + ":" + std::to_string(value) + " | ";
        }

        void log(string item, double value){
            string tempSend;
            toSend = toSend + item + ":" + std::to_string(value) + " | ";
        }

        void log(string item){
            string tempSend;
            toSend = toSend + item + " | ";
        }

        void tick(){
            char send[toSend.length() + 1];
            printf("%s", send);
            toSend = "";
        }

        void loggerThread(){
            while(1){
                tick();
                ThisThread::sleep_for(delay_ms);
            }
        }
};

#endif