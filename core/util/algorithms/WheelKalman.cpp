//
// Created by ben332004 on 2/22/23.
//

#include "WheelKalman.h"

WheelKalman::WheelKalman() {
    this->dt = 0;

    this->setP(0, 0, 0.1);
    this->setP(1, 1, 0.1);

    this->setQ(0, 0, 0.1);
    this->setQ(1, 1, 0.1);

    this->setR(0, 0, 1);
}

void WheelKalman::setDt(double dt) {
    this->dt = dt;
}

void WheelKalman::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {
    // Process model/state transition function (previous states to current states)
    fx[0] = x[0] + dt * x[1];
    fx[1] = x[1];

//    printf("DT: %i\n", (int) (1000 * dt));
//    printf("X: %i %i\n", (int) (x[0]), (int) (x[1]));

    // Jacobian of process model function
    F[0][0] = 1;
    F[0][1] = dt;
    F[1][1] = 1;

//    printf("Prev states: %i %i %i %I %I\n", (int) x[4] * 10);

    // Measurement function
    hx[0] = x[1];
//    printf("H: %i %i %i %i\n", (int) (hx[0] * 1000), (int) (hx[1] * 1000), (int) (hx[2] * 1000), (int) (hx[3] * 1000), (int) (hx[4] * 1000));

    // Jacobian of measurement function
    H[0][1] = 1;
}
