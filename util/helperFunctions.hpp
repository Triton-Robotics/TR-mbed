#include "mbed.h"
#ifndef helperFunctions_hpp
#define helperFunctions_hpp

/**
 * @brief Maps one range of values to another
 * 
 * @param x the value to be mapped
 * @param in_min the minimum value of the range of input values
 * @param in_max the maximum value of the range of input values
 * @param out_min the minimum value of the range of output values
 * @param out_max the maximum value of the range of output values
 */ 
inline float map(int x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

/**
 * @brief Prints floats using mbed because its not built into mbed
 * 
 * @param input the number to print
 * @param decimals the number of decimal points to keep.
 */ 
inline void printFloat(float input, int decimals) {
    if (input < 0) 
        printf("-");
    input = abs(input);
    printf("%d", int(input));
    printf(".");
    input = input - int(input);
    printf("%d\n", int(input * (pow(10, decimals))));
} 

inline void int16ToBitArray(int n, int binaryNum[])
    {
        for (int i = 0; i < 16; ++i) {      
            binaryNum[15-i] = (n >> i) & 1;
        }

        // for (int i = 0; i < 16; i++) {
        //     printf("%d", binaryNum[i]);
        // }
        // printf("\n");
    }

inline void printArray(int16_t array[], int length){
    for(int i = 0; i < length; i ++){
        printf("%d\t",array[i]);
    }
    printf("\n");
}

inline void printArray(int8_t array[], int length){
    for(int i = 0; i < length; i ++){
        printf("%d\t",array[i]);
    }
    printf("\n");
}

inline void printArray(uint8_t array[], int length){
    for(int i = 0; i < length; i ++){
        printf("%d\t",array[i]);
    }
    printf("\n");
}




#endif