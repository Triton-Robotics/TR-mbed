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
        float lastError = 0;
        float sumError = 0;
    public:

        PID(){
            kP = 1; kI = 0; kD = 0;
            integralCap = 0;
            outputCap = 0;
        }

        /**
         * @brief Simple PID Constructor
         *
         * @param p
         * @param i
         * @param d
         * @param sumCap the integral sum cap, to prevent runaway I values
         *
         * @param outCap a cap on the actual output so you can limit how much the pid will output until you're sure it 
         * works well before you let it loose
         */
        PID(float p, float i, float d, float sumCap = 0, float outCap = 0){
            kP = p; kI = i; kD = d;
            integralCap = sumCap;
            outputCap = outCap;
        }

        /**
         * @brief calculate the PID output
         *
         * @param desiredV the desired value
         * @param actualV the actual value
         * @param dt the change in time
         */
        float calculate(float desiredV, float actualV, float dt){
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
            //ThisThread::sleep_for(1ms); //neccessary or else dt -> 0 and causes issues....
            //printf("desired: %d actual: %d \n",(int)desiredV, int(actualV));

            
            return PIDCalc;
        }

        /**
         * @brief set the integral cap
         * @param sumCap the integral cap
         */
        void setIntegralCap(float sumCap){
            integralCap = sumCap;
        }

        /**
         * @brief set the output cap
         * @param outCap the output cap
         */
        void setOutputCap(float outCap){
            outputCap = outCap;
        }

        /**
         * @brief set the PID values
         * @param p
         * @param i
         * @param d
         */
        void setPID(float p, float i, float d){
            kP = p; kI = i; kD = d;
        }
};

#endif //pid_h