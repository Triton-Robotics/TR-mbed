//
// Created by ben332004 on 2/11/23.
//

#include "ChassisKalman.h"


ChassisKalman::ChassisKalman() {        // Process noise
    this->dt = 0;

    this->setP(0, 0, 0.01);
    this->setP(1, 1, 0.01);
    this->setP(2, 2, 0.01);
    this->setP(3, 3, 0.01);
    this->setP(4, 4, 0.01);
    this->setP(5, 5, 0.01);

    this->setQ(0, 0, 0.01);
    this->setQ(1, 1, 0.01);
    this->setQ(2, 2, 0.01);
    this->setQ(3, 3, 0.01);
    this->setQ(4, 4, 0.01);
    this->setQ(5, 5, 0.01);

    // Measurement noise
    this->setR(0, 0, 0.05);
    this->setR(1, 1, 0.05);
    this->setR(2, 2, 0.05);
    this->setR(3, 3, 0.05);
    this->setR(4, 4, 0.05);
}

void ChassisKalman::setDt(double dt) {
    this->dt = dt;
}

double ChassisKalman::pythag(double dx, double dy) {
    return sqrt(dx * dx + dy * dy);
}

double ChassisKalman::degreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

void ChassisKalman::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) {
    // Process model/state transition function (previous states to current states)
    fx[0] = x[0] + dt * x[1];
    fx[1] = x[1];
    fx[2] = x[2] + dt * x[3];
    fx[3] = x[3];
    fx[4] = x[4] + dt * x[5];
    fx[5] = x[5];
    
//    printf("X: %i %i %i %i %i %i\n", (int) (x[0] * 1000), (int) (x[1] * 1000), (int) (x[2] * 1000), (int) (x[3] * 1000), (int) (x[4] * 1000), (int) (x[5] * 1000));

    // Jacobian of process model function
    F[0][0] = 1;
    F[0][1] = dt;
    F[1][1] = 1;
    F[2][2] = 1;
    F[2][3] = dt;
    F[3][3] = 1;
    F[4][4] = 1;
    F[4][5] = dt;
    F[5][5] = 1;

//    printf("Prev states: %i %i %i %I %I\n", (int) x[4] * 10);

    double cosAngle = cos(degreesToRadians(x[4]));
    double sinAngle = sin(degreesToRadians(x[4]));
    double robotRelativeXVel = x[1] * cosAngle + x[3] * sinAngle;
    double robotRelativeYVel = -x[1] * sinAngle + x[3] * cosAngle;
    double rotationVel = x[5] * TRACK_WIDTH_INCHES / 2.0;

    // Measurement function
    hx[0] = robotRelativeXVel + robotRelativeYVel + rotationVel;
    hx[1] = robotRelativeXVel - robotRelativeYVel + rotationVel;
    hx[2] = -robotRelativeXVel + robotRelativeYVel + rotationVel;
    hx[3] = -robotRelativeXVel - robotRelativeYVel + rotationVel;
    hx[4] = x[4];
//    printf("H: %i %i %i %i\n", (int) (hx[0] * 1000), (int) (hx[1] * 1000), (int) (hx[2] * 1000), (int) (hx[3] * 1000), (int) (hx[4] * 1000));

    // Jacobian of measurement function
    H[0][1] = (cosAngle - sinAngle);
    H[0][3] = (cosAngle + sinAngle);
    H[0][4] = (x[3] - x[1]) * degreesToRadians(cosAngle) - (x[1] + x[3]) * degreesToRadians(sinAngle);
    H[0][5] = TRACK_WIDTH_INCHES / 2.0;

    H[1][1] = (cosAngle + sinAngle);
    H[1][3] = (sinAngle - cosAngle);
    H[1][4] = (x[1] + x[3]) * degreesToRadians(cosAngle) + (x[3] - x[1]) * degreesToRadians(sinAngle);
    H[1][5] = TRACK_WIDTH_INCHES / 2.0;

    H[2][1] = (-cosAngle - sinAngle);
    H[2][3] = (cosAngle - sinAngle);
    H[2][4] = -(x[1] + x[3]) * degreesToRadians(cosAngle) + (x[1] - x[3]) * degreesToRadians(sinAngle);
    H[2][5] = TRACK_WIDTH_INCHES / 2.0;

    H[3][1] = (sinAngle - cosAngle);
    H[3][3] = (-cosAngle - sinAngle);
    H[3][4] = (x[1] - x[3]) * degreesToRadians(cosAngle) + (x[1] + x[3]) * degreesToRadians(sinAngle);
    H[3][5] = TRACK_WIDTH_INCHES / 2.0;

    H[4][4] = 1;
}

