#pragma once
#include "mbed.h"

class Subsystem
{
    public:
    void periodic();
    void getState();
    void setState();
};