#ifndef TR_EMBEDDED_MUTEX_H
#define TR_EMBEDDED_MUTEX_H

#include "mbed.h"
#include <cstdlib>
#include <queue>

class TRMutex {

public:
    Thread print_code_thread;

    TRMutex();
    
    void print(int integer);

    void print(char statement[]);

//---- println
    void println(int integer);

    void println(char statement[]);

//---- printf

    void printff(const char* format, ...);

    static void loop();

};

#endif //TR_EMBEDDED_MUTEX_H