//
// Created by ben332004 on 3/13/23.
//

#include "Pose2D.h"

Pose2D::Pose2D(double x, double y, double angleRadians) {
    this->x = x;
    this->y = y;
    this->angleRadians = angleRadians;
}

Pose2D Pose2D::minus(Pose2D other) {
    return {
            x - other.x,
            y - other.y,
            angleRadians - other.angleRadians
    };
}

Pose2D Pose2D::interpolate(Pose2D other, double fractionThis) {
    return {
            other.x + fractionThis * (x - other.x),
            other.y + fractionThis * (y - other.y),
            other.angleRadians + fractionThis * (angleRadians - other.angleRadians),
    };
}
