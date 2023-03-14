//
// Created by ben332004 on 3/13/23.
//

#include "RamseteController.h"

RamseteController::RamseteController() {
    b = 2.0;
    zeta = 0.7;
}

ChassisSpeeds RamseteController::calculate(Pose2D currentPose, Pose2D desiredPose, double desiredLinearVelocity,
                                           double desiredAngularVelocity) {
    Pose2D poseError = desiredPose.minus(currentPose);

    double k = 2.0 * zeta * sqrt(desiredAngularVelocity * desiredAngularVelocity + b * desiredLinearVelocity * desiredLinearVelocity);

    return {
        desiredLinearVelocity * cos(poseError.angleRadians) + k * poseError.x,
        0,
        desiredAngularVelocity + k * poseError.angleRadians + b * desiredLinearVelocity * sinc(poseError.angleRadians) * poseError.y
    };
}

double RamseteController::sinc(double x) {
    if (abs(x) < 1e-9) {
        return 1.0 - 1.0 / 6.0 * x * x;
    }
    return sin(x) / x;
}