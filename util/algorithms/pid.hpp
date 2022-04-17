#include "mbed.h" // I would remove this but I think somehow it supports std::max and std::min
#include <cstdlib>
#ifndef pid_hpp
#define pid_hpp
class PID {
    private:
        float kP;
        float kI;
        float kD;
        float integralCap;
        float outputCap;
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
        PID(float p, float i, float d, float sumCap = 0, float outCap = 0){
            kP = p; kI = i; kD = d;
            integralCap = sumCap;
            outputCap = outCap;
        }

        float calculate(float desiredV, float actualV, float dt){
            static float lastError = 0;
            static float sumError = 0;
            float error = (desiredV - actualV);
            float PIDCalc = kP * error + kI * sumError + kD * ((double)(error - lastError)/dt);
            sumError += error;
            lastError = error;
            
            if(integralCap != 0){
                sumError = std::max(std::min(sumError,integralCap),-integralCap);
            }
            if(outputCap != 0){
                PIDCalc = std::max(std::min(PIDCalc,outputCap),-outputCap);
            }

            printf("desired: %d actual: %d \n",(int)desiredV, int(actualV));

            return PIDCalc;
        }

        void setIntegralCap(float sumCap){
            integralCap = sumCap;
        }

        void setOutputCap(float outCap){
            outputCap = outCap;
        }

        void setPID(float p, float i, float d){
            kP = p; kI = i; kD = d;
        }
};

#endif //pid_h