#include "mbed.h"

class Robot {

    public :
        virtual int execute() = 0;
        virtual int finish() = 0;
        virtual void log() = 0;

};