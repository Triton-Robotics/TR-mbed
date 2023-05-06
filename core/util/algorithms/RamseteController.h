//
// Created by ben332004 on 3/13/23.
//

#ifndef TR_EMBEDDED_RAMSETECONTROLLER_H
#define TR_EMBEDDED_RAMSETECONTROLLER_H

#include <algorithms/ChassisSpeeds.h>
#include <algorithms/Pose2D.h>
#include <math.h>

/**
 * The RamseteController class is used to implement the Ramsete control algorithm for path following, used by the Chassis
 * to autonomously follow a certain path.
 */
class RamseteController {
private:
    double b;
    double zeta;

    double sinc(double x);

public:
    /**
     * Creates a new RamseteController with the default control parameters
     */
    RamseteController();

    /**
     * Calculates the optimal speeds to move the Chassis from its current position to a new position and acheive a certain velocity.
     *
     * @param currentPose The Chassis's current position
     * @param desiredPose The desired position to move the Chassis to
     * @param desiredLinearVelocity The desired linear velocity of the Chassis, in RPM
     * @param desiredAngularVelocity The desired angular velocity of the Chassis, in radians per second
     * @return
     */
    ChassisSpeeds calculate(Pose2D currentPose, Pose2D desiredPose, double desiredLinearVelocity, double desiredAngularVelocity);
};


#endif //TR_EMBEDDED_RAMSETECONTROLLER_H
