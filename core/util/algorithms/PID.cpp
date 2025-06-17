#include "PID.h"

PID::PID()
{
    kP = 0;
    kI = 0;
    kD = 0;
    integralCap = 0;
    outputCap = 0;
    feedForward = 0;
}

PID::PID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;
}

void PID::setPID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;
}

void PID::resetPID(float kP, float kI, float kD, float integralCap, float outputCap)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integralCap = integralCap;
    this->outputCap = outputCap;

    lastError = 0;
    lastError1 = 0;
    lastError2 = 0;
    lastError3 = 0;
    lastError4 = 0;
    lastError5 = 0;
    errorIntegral = 0;
}

void PID::resetErrorIntegral()
{
    lastError = 0;
    lastError1 = 0;
    lastError2 = 0;
    lastError3 = 0;
    lastError4 = 0;
    lastError5 = 0;
    errorIntegral = 0;
}

int PID::calculate(int desired, int current, double dt)
{
    float error = static_cast<float>(desired - current);
    return calculatePeriodic(error, dt);
}

int PID::calculatePeriodic(float error, double dt)
{

    dt /= 1000;
    errorIntegral += kI * dt * (error + lastError) / 2;
    limitErrorIntegral();

    iC = errorIntegral;

    double PIDCalc = (kP * error) + (errorIntegral) + feedForward;
    pC = kP * error;

    if (dt > 0)
    {
        PIDCalc += (kD * (error - lastError) / dt);
    }

    dC = (kD * (error - lastError) / dt);

    //lastError = error;
    lastError1 = error;
    lastError2 = lastError1;
    lastError3 = lastError2;
    lastError4 = lastError3;
    lastError5 = lastError4;
    lastError = (lastError1 + lastError2 + lastError3 + lastError4 + lastError5) / 5;
    limitOutput(PIDCalc);
    return static_cast<int>(PIDCalc);
}

void PID::limitErrorIntegral()
{
    if (integralCap != 0)
    {
        if (errorIntegral > integralCap)
            errorIntegral = integralCap;

        else if (errorIntegral < -integralCap)
            errorIntegral = -integralCap;
    }
}

void PID::limitOutput(double &PIDCalc) const
{

    if (outputCap != 0)
    {
        if (PIDCalc > outputCap)
            PIDCalc = outputCap;

        else if (PIDCalc < -outputCap)
            PIDCalc = -outputCap;
    }
}

void PID::setIntegralCap(float integralCap)
{
    this->integralCap = integralCap;
}

void PID::setOutputCap(float outputCap)
{
    this->outputCap = outputCap;
}