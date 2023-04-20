#include "TRMutex.h"
#include "mbed.h"
#include <cstdlib>
#include <queue>

static Mutex MUTEX;
static std::queue<char> buffer;


TRMutex::TRMutex() {
    print_code_thread.start(loop);
}

//----print no line

//string/char inputs
void TRMutex::print(char statement[]) {
    int length = sizeof(statement)/sizeof(char);
        for (int i = 0; i < length; i++) {
            buffer.push(statement[i]);
        }
}

//ints
void TRMutex::print(int integer) {
    char temp[11];
    sprintf(temp, "%d", integer);
    print(temp);
}

//------println 

//string/char inputs
void TRMutex::println(char statement[]) {
    int length = sizeof(statement)/sizeof(char);
        for (int i = 0; i < length; i++) {
            buffer.push(statement[i]);
        }
        buffer.push('\n');
}

//ints
void TRMutex::println(int integer) {
    char temp[11];
    sprintf(temp, "%d", integer);
    print(temp);
    buffer.push('\n');
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


