//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_PID_H
#define TR_EMBEDDED_PID_H

#include "mbed.h"
#define lastX 5

struct lastfew {
    long long last[lastX] = { 0 };
    int arm = 0;

    void add(long long l) {
        last[arm] = l;
        arm++;
        if (arm == lastX)
            arm = 0;
    }

    long long time() {
        long long t = 0;
        for (int i = 0; i < lastX; i++)
            t += last[i];
        return t / lastX;
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
    float outputCap = 0;

    float lastError = 0;
    double errorIntegral = 0;
    lastfew dBuffer;

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
    int lastY = 0;

    lastfew(int lastY_ = 0) {
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
        for (int i = 0; i < lastX; i++)
            t += last[i];
        return t / lastX;
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
    float outputCap = 0;

    float lastError = 0;
    double errorIntegral = 0;

public:
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