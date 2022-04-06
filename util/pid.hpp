#include "mbed.h"
#ifndef pid_hpp
#define pid_hpp
class PID {
    private:
        int kP;
        int kI;
        int kD;
        int integralCap;
    public:
        PID(int p, int i, int d){
            kP = p; kI = i; kD = d;
            integralCap = 0;
        }
        int calculate(int desiredV, int actualV, int dt){
            static int lastError = 0;
            static int sumError = 0;
            int error = (desiredV - actualV);
            int PIDCalc = kP * error + kI * sumError + kD * ((double)(error - lastError)/dt);
            sumError += error;
            lastError = error;
            if(integralCap != 0){
                sumError = std::max(std::min(sumError,integralCap),-integralCap);
            }
            return PIDCalc;
        }
};

#endif //motor_h