#include "mbed.h"

class Subsystem
{
    public:
    void init();
    void periodic();
    void getState();
    void setState();
};