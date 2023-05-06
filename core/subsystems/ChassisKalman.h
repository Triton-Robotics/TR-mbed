//
// Created by ben332004 on 2/11/23.
//

#ifndef TR_EMBEDDED_CHASSISKALMAN_H
#define TR_EMBEDDED_CHASSISKALMAN_H

#define Nsta 6 // 6 states: x, y, and rotation pos and velocity
#define Mobs 5 // 5 Measurements: velocity of each wheel, and IMU angle

#include <TinyEKF.h>
#include <cmath>

#define PI 3.14159265
#define SECONDS_PER_MINUTE 60
#define TICKS_PER_ROTATION 8192.0
#define M3508_GEAR_RATIO 19.0
#define WHEEL_DIAMETER_INCHES 4.615
#define TRACK_WIDTH_INCHES 18.0


/**
 * Kalman filter for Chassis localization.
 *
 * States are:
 * - x position (inches)
 * - x velocity (inches / second)
 * - y position (inches)
 * - y velocity (inches / second)
 * - heading (degrees)
 * - rotation (degrees / second)
 *
 * Measurements are:
 * - LF velocity (inches / second)
 * ... veloctities for RF, LB, and RB
 *
 * - IMU angle (degrees)
 */
class ChassisKalman : public TinyEKF {
public:
    ChassisKalman();

    void setDt(double dt);

    double dt;

protected:
    double pythag(double dx, double dy);
    double degreesToRadians(double degrees);

    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) override;
};


#endif //TR_EMBEDDED_CHASSISKALMAN_H
