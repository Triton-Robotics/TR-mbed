//
// Created by ankit on 1/31/23.
//

#include "PID.h"

PID::PID(){
    kP = 1; kI = 0; kD = 0;
    integralCap = 0;
    outputCap = 0;
    feedForward = 0;
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
PID::PID(float p, float i, float d, float sumCap = 0, float outCap = 0){
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
float PID::calculate(float desiredV, float actualV, double dt){
    dt /= 1000;
//    printf("dt (ms): %i\n", (int) (1000 * dt));

    float error = (desiredV - actualV);
    float PIDCalc = kP * error /*+ kI * sumError*/ + kD * ((double)(error - lastError)/dt);
//    if (error < 6 & error > -6) {
        sumError += error * dt;
//    } else {
//        sumError = 0;
//    }
    lastError = error;

    if(integralCap != 0){
        //sumError = std::max(std::min(sumError,integralCap),-integralCap);
        if(sumError > integralCap)
            sumError = integralCap;
        else if(sumError < -integralCap)
            sumError = -integralCap;
    }
    if(outputCap != 0){
        //PIDCalc = std::max(std::min(PIDCalc,outputCap),-outputCap);
        if(PIDCalc > outputCap)
            PIDCalc = outputCap;
        else if(PIDCalc < -outputCap)
            PIDCalc = -outputCap;
    }
//    ThisThread::sleep_for(1ms); //neccessary or else dt -> 0 and causes issues....
    if(debug)
        printf("DES: %d ACT: %d PID: %d\n",(int)desiredV, int(actualV), int(PIDCalc));

    return PIDCalc + feedForward;
}

/**
         * @brief set the integral cap
         * @param sumCap the integral cap
         */
void PID::setIntegralCap(float sumCap){
    integralCap = sumCap;
}

/**
 * @brief set the output cap
 * @param outCap the output cap
 */
void PID::setOutputCap(float outCap){
    outputCap = outCap;
}

/**
 * @brief set the PID values
 * @param p
 * @param i
 * @param d
 */
void PID::setPID(float p, float i, float d){
    kP = p; kI = i; kD = d;
}

int PID::getkP(){
    return kP;
}

int PID::getkI(){
    return kI;
}

int PID::getkD(){
    return kD;
}