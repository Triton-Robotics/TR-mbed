//
// Created by ben332004 on 2/22/23.
//

#include "WheelKalman.h"

WheelKalman::WheelKalman() {
    this->dt = 0;

    this->setP(0, 0, 1000);
    this->setP(1, 1, 1000);

    this->setQ(0, 0, 1000);
    this->setQ(1, 1, 1000);


    this->setR(0, 0, 100);
//    this->setR(0, 1, 10);
//    this->setR(1, 0, 10);
    this->setR(1, 1, 500);

//    this->u[0] = 0;
//    this->B[0][0] = 0;
//    this->B[1][0] = 10;
}

void WheelKalman::setDt(double dt) {
    this->dt = dt;
}

void WheelKalman::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {
    // Process model/state transition function (previous states to current states)
    fx[0] = x[0] + dt * x[1];
    fx[1] = x[1];

    for (int i = 0; i < Nsta; i++) {
        for (int j = 0; j < ControlInputs; j++) {
            fx[i] += B[i][j] * u[j];
        }
    }

//    printf("DT: %i\n", (int) (1000 * dt));
//    printf("X: %i %i\n", (int) (x[0]), (int) (x[1]));

    // Jacobian of process model function
    F[0][0] = 1;
    F[0][1] = dt;
    F[1][1] = 1;

//    printf("Prev states: %i %i %i %I %I\n", (int) x[4] * 10);

    // Measurement function
    hx[0] = x[0];
    hx[1] = x[1];
//    printf("H: %i %i %i %i\n", (int) (hx[0] * 1000), (int) (hx[1] * 1000), (int) (hx[2] * 1000), (int) (hx[3] * 1000), (int) (hx[4] * 1000));

    // Jacobian of measurement function
    H[0][0] = 1;
    H[1][1] = 1;
}
