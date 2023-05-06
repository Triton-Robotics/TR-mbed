//
// Created by ben332004 on 3/13/23.
//

#ifndef TR_EMBEDDED_POSE2D_H
#define TR_EMBEDDED_POSE2D_H


/**
 * The Pose2D class represents a 2D position and orientation, including x and y coordinates and a heading. It is used
 * to represent the Chassis's position and heading.
 */
class Pose2D {

public:
    /**
     * Creates a new Pose2D object from the specified position and orientation
     *
     * @param x The x-coordinate of the position in meters
     * @param y The y-coordinate of the position in meters
     * @param angleRadians The CCW-positive angle in radians
     */
    Pose2D(double x, double y, double angleRadians);

    /**
     * Subtracts another position and orientation from this position and orientation
     * @param other The Pose2D to subtract
     * @return A new Pose2D representing the difference between this Pose2D and the other Pose2d
     */
    Pose2D minus(Pose2D other);

    /**
     * Finds a new Pose2D that is a specific fraction of the way between this Pose2D and another. For example, if this
     * Pose2D is x = 1, y = 4, rotation = PI/2, and the other Pose2D is x = 3, y = 3, rotation = PI/6, then interpolating
     * with fractionThis = 1/4 would be x = 2.5, y = 3.25, rotation = PI/4.
     *
     * @param other The Pose2D to interpolate from
     * @param fractionThis What fraction (0 to 1) of the way from the other Pose2D to this Pose2D; 1 would just return
     * this Pose2D, and 0 would return the other Pose2D
     * @return The Pose2D that is the specified fraction between this and the other Pose2D
     */
    Pose2D interpolate(Pose2D other, double fractionThis);

    double x;
    double y;
    double angleRadians;
};


#endif //TR_EMBEDDED_POSE2D_H
