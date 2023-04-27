//
// Created by ben332004 on 3/13/23.
//

#ifndef TR_EMBEDDED_POSE2D_H
#define TR_EMBEDDED_POSE2D_H


class Pose2D {

public:
    Pose2D(double x, double y, double angleRadians);

    Pose2D minus(Pose2D other);

    Pose2D interpolate(Pose2D other, double fractionThis);

    double x;
    double y;
    double angleRadians;
};


#endif //TR_EMBEDDED_POSE2D_H
