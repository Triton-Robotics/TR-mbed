#pragma once
#include "mbed.h"

class Sensor  
{
public:
    void init();
    void read();
    void reset();
};