//
// Created by ben332004 on 2/22/23.
//

#ifndef TR_EMBEDDED_WHEELKALMAN_H
#define TR_EMBEDDED_WHEELKALMAN_H

#define Nsta 2 // 2 states: rotational angle and velocity
#define Mobs 2 // 2 measurements: rotational angle and velocity
#define ControlInputs 1 // 1 control input: power sent to motor

#include <TinyEKF.h>
#include <cmath>

#define SECONDS_PER_MINUTE 60
#define TICKS_PER_ROTATION 8192.0
#define M3508_GEAR_RATIO 19.0
#define WHEEL_DIAMETER_INCHES 5.0
#define TRACK_WIDTH_INCHES 18.0


/**
 * The WheelKalman class is a Kalman filter used to control a single wheel/motor. It uses an Extended Kalman Filter (EKF)
 * to implement position control of the wheel.
 */
class WheelKalman: public TinyEKF {
public:
    /**
     * Creates a new WheelKalman object
     */
    WheelKalman();

    /**
     * Sets the dt (time delta) of the kalman filter, used to predict how much the position will change each timestep
     * based on speed and dt
     * @param dt The time delta in milliseconds
     */
    void setDt(double dt);

    double dt;
    double u[ControlInputs];

protected:
    double B[Nsta][ControlInputs];

    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) override;
};


#endif //TR_EMBEDDED_WHEELKALMAN_H
