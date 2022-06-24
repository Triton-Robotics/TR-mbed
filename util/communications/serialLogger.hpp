#include "mbed.h"
#include <string>

#define MAX_PRINT_CHARS 200 //Number of can handlers
#ifndef seriallogger_hpp
#define seriallogger_hpp

using namespace std;

class Logger{
    private:
        int index = 0;
        string toSend = "";

    public:

        
        void log(string str){
            toSend += str;
        }

        void tick(){
            printf(toSend);
            toSend = "";
        }
};

#endif