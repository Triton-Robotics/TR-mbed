#include "TRMutex.h"
#include "mbed.h"
#include <cstdlib>
#include <queue>

static Mutex MUTEX;
static std::queue<char> buffer;


TRMutex::TRMutex() {
    print_code_thread.start(loop);
}


void TRMutex::printMutex(char statement[]) {
    int length = sizeof(statement)/sizeof(char);
        for (int i = 0; i < length; i++) {
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
        MUTEX.unlock();
        ThisThread::sleep_for(1ms);
    }
}


