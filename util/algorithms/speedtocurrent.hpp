#include "mbed.h"
#include <cmath>

//new ln function! actually uses (0,0) except ln doesnt have zero so do that manually
//f(x) = 1539.8390ln(x) - 3511.0732 
//g(x) = 89.4056ln(x) - 212.7353

inline int M3508speedtocurrent(int desSpeed) {
    // if (desSpeed < 316)
    //     return 0;
    // else if (desSpeed < 9700) {
    //     int x = desSpeed;
    //     return 219+0.248*x + -6.97*pow(10,-5)*x*x + 8.15*pow(10,-9)*x*x*x + -3.28*pow(10,-13)*x*x*x*x;
    // }
    // else 
    //     return 15000;

    if (desSpeed <= 20)
        return 0;
    else{
        return desSpeed * 89.4056 * log(desSpeed) - desSpeed - 212.7353 * desSpeed;
    }    
}