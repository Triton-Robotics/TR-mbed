//
// Created by ben332004 on 2/22/23.
//

#ifndef TR_EMBEDDED_WHEELKALMAN_H
#define TR_EMBEDDED_WHEELKALMAN_H

#define Nsta 2 // 3 states: rotational angle and velocity
#define Mobs 2 // 2 measurements: rotational angle and velocity
#define ControlInputs 1 // 1 control input: power sent to motor

#include <TinyEKF.h>
#include <cmath>

#define SECONDS_PER_MINUTE 60
#define TICKS_PER_ROTATION 8192.0
#define M3508_GEAR_RATIO 19.0
#define WHEEL_DIAMETER_INCHES 5.0
#define TRACK_WIDTH_INCHES 18.0



class WheelKalman: public TinyEKF {
public:
    WheelKalman();

    void setDt(double dt);

    double dt;
    double u[ControlInputs];

protected:
    double B[Nsta][ControlInputs];

    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) override;
};


#endif //TR_EMBEDDED_WHEELKALMAN_H
