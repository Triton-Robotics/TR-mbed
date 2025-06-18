//
// Created by ben332004 on 3/13/23.
//

#ifndef TR_EMBEDDED_WHEELSPEEDS_H
#define TR_EMBEDDED_WHEELSPEEDS_H


/**
 * The WheelSpeeds class represents the speeds of the 4 chassis motors. It is used by the Chassis to drive the 4 motors
 * at certain speeds.
 */
class WheelSpeeds {
public:
    /**
     * Creates a new WheelSpeeds object with the given speeds of each motor
     *
     * @param LF The speed of the left front motor in RPM
     * @param RF The speed of the right front motor in RPM
     * @param LB The speed of the left back motor in RPM
     * @param RB The speed of the right back motor in RPM
     */
    WheelSpeeds(double LF, double RF, double LB, double RB);

    double LF;
    double LB;
    double RF;
    double RB;
};


#endif //TR_EMBEDDED_WHEELSPEEDS_H
