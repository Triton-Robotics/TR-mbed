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

    double linearResult = desiredLinearVelocity * cos(poseError.angleRadians) + k * poseError.x;
    if (linearResult > 1000) {
        linearResult = 1000;
    }
    double angularResult = desiredAngularVelocity + k * poseError.angleRadians + b * desiredLinearVelocity * sinc(poseError.angleRadians) * poseError.y;
    if (angularResult > 1000) {}
    angularResult = 1000;
    return {
        linearResult,
        0,
        angularResult
    };
}

double RamseteController::sinc(double x) {
    if (abs(x) < 1e-9) {
        return 1.0 - 1.0 / 6.0 * x * x;
    }
    return sin(x) / x;
}