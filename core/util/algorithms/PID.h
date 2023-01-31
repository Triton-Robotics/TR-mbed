//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_PID_H
#define TR_EMBEDDED_PID_H

#include "mbed.h"

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
    bool debug = false;
    float feedForward = 0;

    // Methods

    PID();
    PID(float p, float i, float d, float sumCap, float outCap);

    float calculate(float desiredV, float actualV, float dt);

    void setIntegralCap(float sumCap);
    void setOutputCap(float outCap);
    void setPID(float p, float i, float d);

    int getkP();
    int getkI();
    int getkD();
};


#endif //TR_EMBEDDED_PID_H
