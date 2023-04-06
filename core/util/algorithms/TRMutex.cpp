#ifndef TR_EMBEDDED_MUTEX_C
#define TR_EMBEDDED_MUTEX_C
#include "TRMutex.h"
#include "mbed.h"
#include <cstdlib>
// constructing queues
#include <iostream>       // std::cout
#include <list>           // std::list
#include <queue>


TRMutex::TRMutex() {
    print_code_thread.start(loop);
    
}


string TRMutex::printMutex(string statement) {
    for (int i = 0; i < statement.length(); i++) {
        buffer.push(statement[i]);
    }
    
}

void TRMutex::loop() {
    while (true) {
        MUTEX.lock();

        if (!buffer.empty()) {
            printf("%c", buffer.front());
            buffer.pop();
        }
        
        ThisThread::sleep_for(1ms);
        MUTEX.unlock();
    }
}

#endif //TR_EMBEDDED_MUTEX_C
