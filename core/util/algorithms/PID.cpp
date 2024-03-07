#include "PID.h"
#include <cmath>

PID::PID(){
    kP = 0; kI = 0; kD = 0;
    integralCap = 0;
    outputCap = 0;
    feedForward = 0;

    pComponent = 0;
    iComponent = 0;
    dComponent = 0;
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

    double PIDCalc = (kP * error) + (kI * errorIntegral) + feedForward;

    if(dt > 0)
        PIDCalc += (kD * (error - lastError) / dt);

    pComponent = kP * error;
    iComponent = kI * errorIntegral;
    dComponent = (kD * (error - lastError) / dt);

    lastError = error;

    limitOutput(PIDCalc);
    return static_cast<int>(PIDCalc);
}

int PID::calculatePeriodic(float error, double dt) {

    dt /= 1000;
    errorIntegral += kI * dt * (error + lastError) / 2;

    double PIDCalc = (kP * error) + (errorIntegral) + feedForward;

    if(dt > 0) {
        PIDCalc += (kD * (error - lastError) / dt);
    }

    pComponent = kP * error;
    iComponent = errorIntegral;
    dComponent = (kD * (error - lastError) / dt);

    lastError = error;
    limitOutput(PIDCalc);
    return static_cast<int>(PIDCalc);
}

void PID::limitOutput(double &PIDCalc){

    if(integralCap != 0) {
        if (errorIntegral > integralCap) {
            errorIntegral = integralCap;
            //printf("%f\n", errorIntegral);
        }
            

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

// line 32
// capped at 200, then 200 * kI
// same result as (2/0.01)

// line 54
// issue: both functions intend to do same thing
//          however, input value is not standardized
// potential solution: have 1st function call 2nd
//                      determine error, pass through to 2nd

// capped at 200 * kI