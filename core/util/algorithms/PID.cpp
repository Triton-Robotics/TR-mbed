#include "PID.h"
#include <cmath>

PID::PID(){
    kP = 0; kI = 0; kD = 0;
    integralCap = 0;
    outputCap = 0;
    feedForward = 0;
}


PID::PID(float kP, float kI, float kD, float integralCap, float outputCap){
    this -> kP = kP;
    this -> kI = kI;
    this -> kD = kD;
    this -> integralCap = integralCap;
    this -> outputCap = outputCap;
}

void PID::setPID(float kP, float kI, float kD, float integralCap, float outputCap){
    this -> kP = kP;
    this -> kI = kI;
    this -> kD = kD;
    this -> integralCap = integralCap;
    this -> outputCap = outputCap;
}

int PID::calculate(int desired, int current, double dt) {

    dt /= 1000;

    float error = static_cast<float>(desired - current);
    errorIntegral += dt * (error + lastError) / 2;

    double PIDCalc = (kP * error) + (kI * errorIntegral) + (kD * (error - lastError) / dt) + feedForward;
    lastError = error;

    limitOutput(PIDCalc);
    return static_cast<int>(PIDCalc);
}

int PID::calculatePeriodic(float error, double dt) {

    dt /= 1000;
    errorIntegral += dt * (error + lastError) / 2;

    double PIDCalc = (kP * error) + (kI * errorIntegral) + (kD * (error - lastError) / dt) + feedForward;
    lastError = error;

    limitOutput(PIDCalc);
    return static_cast<int>(PIDCalc);
}

int PID::limitOutput(double &PIDCalc){

    if(integralCap != 0) {
        if (errorIntegral > integralCap)
            errorIntegral = integralCap;

        else if (errorIntegral < -integralCap)
            errorIntegral = -integralCap;
    }

    if(outputCap != 0) {
        if (PIDCalc > outputCap)
            PIDCalc = outputCap;

        else if (PIDCalc < -outputCap)
            PIDCalc = -outputCap;
    }
}

void PID::setIntegralCap(float integralCap){
    this -> integralCap = integralCap;
}

void PID::setOutputCap(float outputCap){
    this -> outputCap = outputCap;
}