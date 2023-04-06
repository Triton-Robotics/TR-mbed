#ifndef TR_EMBEDDED_MUTEX_H
#define TR_EMBEDDED_MUTEX_H

#include "mbed.h"
#include <cstdlib>
// constructing queues
#include <iostream>       // std::cout
#include <list>           // std::list
#include <queue>

class TRMutex {

public:
    TRMutex();

    string printMutex(string statement);

    static void loop();

};

#endif //TR_EMBEDDED_MUTEX_H