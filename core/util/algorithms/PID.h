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
    double pComponent;
    double iComponent;
    double dComponent;
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
