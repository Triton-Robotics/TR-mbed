#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-use-auto"
//
// Created by ankit on 1/31/23.
//

#include "PID.h"
#include <math.h>

PID::PID(){
    kP = 0; kI = 0; kD = 0;
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

    if(lastError == error && carryCount < slopeCarry){
        carryCount ++;
    }else{
        prevRealError = lastError;
        carryCount = 0;
    }

    float PIDCalc = kP * error + kI * sumError + kD * ((double)(error - prevRealError) * (pow(10,d10xMultiplier)/dt));
    if(debugPIDterms)
        printf("P: %f\t I: %f\t D: %f\t\n", kP * error, kI * sumError, kD * ((double)(error - lastError) * (pow(10,d10xMultiplier)/dt)));
    
    //printf("dd[%f]\n",((double)(error - lastError)));

    sumError += error * dt;
    lastError = error;

    if(debug)
        printf("DES: %d\t ACT: %d\t PID: %d\t ERROR: %d\n",(int)desiredV, int(actualV), int(PIDCalc), int(error));

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

    

    return PIDCalc + feedForward;
}

int PID::calculate(int desiredV, int actualV, double dt) {
    dt /= 1000;

    float error = static_cast<float>(desiredV - actualV);

    if(lastError == error && carryCount < slopeCarry){
        carryCount ++;
    }else{
        prevRealError = lastError;
        carryCount = 0;
    }

    double PIDCalc = kP * error + kI * sumError + kD * (error - prevRealError) / dt;
    if(debugPIDterms)
        printf("P: %f\t I: %f\t D: %f\t\n", kP * error, kI * sumError, kD * ((double)(error - lastError) * (pow(10,d10xMultiplier)/dt)));

    //printf("dd[%f]\n",((double)(error - lastError)));

    sumError += error * dt;
    lastError = error;

    if(debug)
        printf("DES: %d\t ACT: %d\t PID: %d\t ERROR: %d\n",(int)desiredV, int(actualV), int(PIDCalc), int(error));

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

    return static_cast<int>(PIDCalc + feedForward);
}


float PID::calculate(float dV, double dt){
    dt /= 1000;
//    printf("dt (ms): %i\n", (int) (1000 * dt));

    float error = dV;

    if(lastError == error && carryCount < slopeCarry){
        carryCount ++;
    }else{
        prevRealError = lastError;
        carryCount = 0;
    }

    float PIDCalc = kP * error + kI * sumError + kD * ((double)(error - prevRealError) * (pow(10,d10xMultiplier)/dt));
    if(debugPIDterms)
        printf("P: %f\t I: %f\t D: %f\t\n", kP * error, kI * sumError, kD * ((double)(error - lastError) * (pow(10,d10xMultiplier)/dt)));

    //printf("dd[%f]\n",((double)(error - lastError)));

    sumError += error * dt;
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

float PID::getkP() const{
    return kP;
}

float PID::getkI() const{
    return kI;
}

float PID::getkD() const{
    return kD;
}

#pragma clang diagnostic pop