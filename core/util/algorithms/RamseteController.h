//
// Created by ben332004 on 3/13/23.
//

#ifndef TR_EMBEDDED_RAMSETECONTROLLER_H
#define TR_EMBEDDED_RAMSETECONTROLLER_H

#include <algorithms/ChassisSpeeds.h>
#include <algorithms/Pose2D.h>
#include <math.h>

class RamseteController {
private:
    double b;
    double zeta;

    double sinc(double x);

public:
    RamseteController();

    ChassisSpeeds calculate(Pose2D currentPose, Pose2D desiredPose, double desiredLinearVelocity, double desiredAngularVelocity);
};


#endif //TR_EMBEDDED_RAMSETECONTROLLER_H
