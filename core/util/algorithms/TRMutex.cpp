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
        int i=0;
        while(statement[i] != '\0') {
            buffer.push(statement[i]);
            i++;
        }
}

//ints
void TRMutex::print(int integer) {
    char temp[11];
    sprintf(temp, "%d", integer);
    print(temp);
}

void TRMutex::printff(const char* format, ...) {
    char temp[50];
    va_list args;
    va_start (args, format);
    vsnprintf (temp, 50, format, args);
    print(temp);
    va_end (args);
    
}

//------println 

//string/char inputs
void TRMutex::println(char statement[]) {
    int i=0;
    while(statement[i] != '\0') {
        buffer.push(statement[i]);
        i++;
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

        if (!buffer.empty()) {
            MUTEX.lock();
            printf("%c", buffer.front());
            buffer.pop();
            MUTEX.unlock();
        }
        
        ThisThread::sleep_for(1ms);
    }
}


