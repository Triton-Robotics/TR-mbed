//
// Created by ben332004 on 3/13/23.
//

#ifndef TR_EMBEDDED_CHASSISSPEEDS_H
#define TR_EMBEDDED_CHASSISSPEEDS_H

/**
 * The ChassisSpeeds class represents the x, y, and rotational speeds of the Chassis, in RPM. ChassisSpeeds objects are
 * used by the Chassis's drive methods.
 */
class ChassisSpeeds {
public:
    /**
     * Creates a new ChassisSpeeds object from the specified speeds.
     *
     * @param x The velocity in the x direction in RPM
     * @param y The velocity in the y direction in RPM
     * @param rotation The rotational velocity in RPM
     */
    ChassisSpeeds(double x, double y, double rotation);

    double x;
    double y;
    double rotation;
};

#endif //TR_EMBEDDED_CHASSISSPEEDS_H
