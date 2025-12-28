//This adds an unstable change to make the D component low pass (avg  itself)
#define FUNNY_PID

#ifndef FUNNY_PID

//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_PID_H
#define TR_EMBEDDED_PID_H

#include "mbed.h"

/**
 * The PID class is used to implement PID (proportional, integral, derivative) control of a motor. It is used by almost every
 * motor on the robot, and supports both position and velocity control.
 */
class PID {
private:
    float kP = 0;
    float kI = 0;
    float kD = 0;

    double integralCap = 0;
    float outputCap = 0;

    float lastError = 0;
    double errorIntegral = 0;

public:
    float feedForward = 0;

    float pC, iC, dC = 0;

    /**
     * Creates a new PID object with the default control parameters
     */
    PID();

    /**
     * Creates a new PID object with the given control parameters
     * @param kP The P (proportional) parameter
     * @param kI The I (integral) parameter
     * @param kD The D (derivative) parameter
     * @param integralCAp The maximum integral that can be achieved, above which the integral will be capped
     * @param outputCap The maximum output of the PID, above which the output will be capped
     */
    PID(float kP, float kI, float kD, float integralCap = 0, float outputCap = 0);

    /**
     * Sets the P, I, and D control parameters
     * @param p The new P (proportional) parameter
     * @param i The new I (integral) parameter
     * @param d The new D (derivative) parameter
     */
    void setPID(float p, float i, float d, float integralCap = 0, float outputCap = 0);

    void resetPID(float kP, float kI, float kD, float integralCap = 0, float outputCap = 0);

    void resetErrorIntegral();


    /**
     * Calculates an output power based on the current and desired measurement (speed/position)
     *
     * @param desired The desired speed/position
     * @param current The current speed/position
     * @param dt The time that has passed since the last calculation, in milliseconds
     * @return The output control power
     */
    int calculate(int desired, int current, double dt);

    int calculatePeriodic(float error, double dt);

    void limitOutput(double &PIDCalc) const;

    void limitErrorIntegral();

    /**
     * Sets the integral cap
     * @param integralCap The new integral cap
     */
    void setIntegralCap(float integralCap);

    /**
     * Sets the output cap
     * @param outCap The new output cap
     */
    void setOutputCap(float outCap);

};

#endif //TR_EMBEDDED_PID_H
#else
//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_PID_H
#define TR_EMBEDDED_PID_H

#include "mbed.h"
#define lastX 10

struct lastfew {
    long long last[lastX] = { 0 };
    int arm = 0;
    int lastY = 1;

    lastfew(int lastY_ = 1) {
        if(lastY_ > lastX){
            lastY = lastX;
        }else{
            lastY = lastY_;
        }
    }

    void add(long long l) {
        last[arm] = l;
        arm++;
        if (arm == lastY)
            arm = 0;
    }

    long long time() {
        long long t = 0;
        for (int i = 0; i < lastY; i++)
            t += last[i];
        return t / lastY;
    }
};

/**
 * The PID class is used to implement PID (proportional, integral, derivative) control of a motor. It is used by almost every
 * motor on the robot, and supports both position and velocity control.
 */
class PID {
private:
    float kP = 0;
    float kI = 0;
    float kD = 0;

    double integralCap = 0;
    double derivativeCap = 0;
    float outputCap = 0;

    float lastError = 0;
    double errorIntegral = 0;

public:

    struct config
    {
        float p = 1;
        float i = 0;
        float d = 0;
        float outCap = 15000;
        float integralCap = 500;
    };


    float feedForward = 0;

    float pC, iC, dC = 0;

    lastfew dBuffer;

    /**
     * Creates a new PID object with the default control parameters
     */
    PID();

    /**
     * Creates a new PID object with the given control parameters
     * @param kP The P (proportional) parameter
     * @param kI The I (integral) parameter
     * @param kD The D (derivative) parameter
     * @param integralCAp The maximum integral that can be achieved, above which the integral will be capped
     * @param outputCap The maximum output of the PID, above which the output will be capped
     */
    PID(float kP, float kI, float kD, float integralCap = 0, float outputCap = 0);

    PID(config cfg);

    /**
     * Sets the P, I, and D control parameters
     * @param p The new P (proportional) parameter
     * @param i The new I (integral) parameter
     * @param d The new D (derivative) parameter
     */
    void setPID(float p, float i, float d, float integralCap = 0, float outputCap = 0);

    void resetPID(float kP, float kI, float kD, float integralCap = 0, float outputCap = 0);

    void resetErrorIntegral();


    /**
     * Calculates an output power based on the current and desired measurement (speed/position)
     *
     * @param desired The desired speed/position
     * @param current The current speed/position
     * @param dt The time that has passed since the last calculation, in milliseconds
     * @return The output control power
     */
    int calculate(int desired, int current, double dt);

    int calculatePeriodic(float error, double dt);

    void limitOutput(double &PIDCalc) const;

    double limitErrorDerivative(double dTerm);

    void limitErrorIntegral();

    /**
     * Sets the integral cap
     * @param derivativeCap The new integral cap
     */
    void setDerivativeCap(float derivativeCap);

    /**
     * Sets the integral cap
     * @param integralCap The new integral cap
     */
    void setIntegralCap(float integralCap);

    /**
     * Sets the output cap
     * @param outCap The new output cap
     */
    void setOutputCap(float outCap);

};



#endif //TR_EMBEDDED_PID_H
#endif