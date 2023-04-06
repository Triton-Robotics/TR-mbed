#ifndef TR_EMBEDDED_MUTEX_H
#define TR_EMBEDDED_MUTEX_H

#include "mbed.h"
#include <cstdlib>
// constructing queues
#include <iostream>       // std::cout
#include <list>           // std::list
#include <queue>

class TRMutex {

private:
    static Mutex MUTEX;
    static Thread print_code_thread;
    static std::queue<char> buffer;


public:
    TRMutex();

    string printMutex(string statement);

    static void loop();

};

#endif //TR_EMBEDDED_MUTEX_H