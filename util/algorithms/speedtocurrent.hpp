#include <cmath>

inline int M3508speedtocurrent(int desSpeed) {
    if (desSpeed < 316)
        return 0;
    else if (desSpeed < 9700) {
        int x = desSpeed;
        return 219+0.248*x + -6.97*pow(10,-5)*x*x + 8.15*pow(10,-9)*x*x*x + -3.28*pow(10,-13)*x*x*x*x;
    }
    else 
        return 15000;
}