#include "mbed.h"
#ifndef helperFunctions_hpp
#define helperFunctions_hpp


inline float map(int x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

inline void printFloat(float input, int decimals) {
    printf("%d", int(input));
    printf(".");
    input = input - int(input);
    printf("%d\n", int(input * (pow(10, decimals))));
} 


#endif