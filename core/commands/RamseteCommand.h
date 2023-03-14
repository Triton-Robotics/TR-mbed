//
// Created by ben332004 on 3/13/23.
//

#ifndef TR_EMBEDDED_RAMSETECOMMAND_H
#define TR_EMBEDDED_RAMSETECOMMAND_H

#include <algorithms/RamseteController.h>
#include <Chassis.h>


class RamseteCommand {
public:
    RamseteCommand(Pose2D startPose, Pose2D endPose, double durationSeconds, Chassis* chassis);

    void initialize();
    void execute();
    void end();
    bool isFinished();

    Pose2D startPose;
    Pose2D endPose;
    Chassis* chassis;

private:
    RamseteController controller;
    double prevTime;
    double durationSeconds;
    double startTime;

};


#endif //TR_EMBEDDED_RAMSETECOMMAND_H
