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
    float kP;
    float kD;
    float integralCap;
    float outputCap;
    float lastError = 0;

public:
    bool debug = false;
    bool debugPIDterms = false;
    float feedForward = 0;

    float kI;
    float sumError = 0;

    int d10xMultiplier = 0;
    int slopeCarry = 5;
    int carryCount = 0;
    double prevRealError = 0;

    /**
     * Creates a new PID object with the default control parameters
     */
    PID();

    /**
     * Creates a new PID object with the given control parameters
     * @param p The P (proportional) parameter
     * @param i The I (integral) parameter
     * @param d The D (derivative) parameter
     * @param sumCap The maximum integral that can be achieved, above which the integral will be capped
     * @param outCap The maximum output of the PID, above which the output will be capped
     */
    PID(float p, float i, float d, float sumCap, float outCap);

    /**
     * Calculates an output power based on the current and desired measurement (speed/position)
     *
     * @param desiredV The desired speed/position
     * @param actualV The current speed/position
     * @param dt The time that has passed since the last calculation, in milliseconds
     * @return The output control power
     */
    float calculate(float desiredV, float actualV, double dt);

    /**
     * Sets the integral cap
     * @param sumCap The new integral cap
     */
    void setIntegralCap(float sumCap);

    /**
     * Sets the output cap
     * @param outCap The new output cap
     */
    void setOutputCap(float outCap);

    /**
     * Sets the P, I, and D control parameters
     * @param p The new P (proportional) parameter
     * @param i The new I (integral) parameter
     * @param d The new D (derivative) parameter
     */
    void setPID(float p, float i, float d);

    /**
     * Gets the P (proportional) control parameter
     * @return The P (proportional) control parameter
     */
    float getkP();

    /**
     * Gets the I (integral) control parameter
     * @return The I (integral) control parameter
     */
    float getkI();

    /**
     * Gets the D (derivative) control parameter
     * @return The D (derivative) control parameter
     */
    float getkD();
};


#endif //TR_EMBEDDED_PID_H
