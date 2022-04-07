#include "mbed.h" // I would remove this but I think somehow it supports std::max and std::min
#ifndef pid_hpp
#define pid_hpp
class PID {
    private:
        int kP;
        int kI;
        int kD;
        int integralCap;
        int outputCap;
    public:

        PID(){
            kP = 1; kI = 0; kD = 0;
            integralCap = 0;
            outputCap = 0;
        }

        /**
         * Simple PID Constructor, has a default P, I, and D parameters
         *
         * Optional Parameters are sumCap which respectively control the integral sum cap, to prevent runaway I values
         *
         * and outCap, a cap on the actual output so you can limit how much the pid will output until you're sure it 
         * works well before you let it loose
         */
        PID(int p, int i, int d, int sumCap = 0, int outCap = 0){
            kP = p; kI = i; kD = d;
            integralCap = sumCap;
            outputCap = outCap;
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
            if(outputCap != 0){
                PIDCalc = std::max(std::min(PIDCalc,outputCap),-outputCap);
            }
            return PIDCalc;
        }

        void setIntegralCap(int sumCap){
            integralCap = sumCap;
        }

        void setOutputCap(int outCap){
            outputCap = outCap;
        }

        void setPID(int p, int i, int d){
            kP = p; kI = i; kD = d;
        }
};

#endif //pid_h