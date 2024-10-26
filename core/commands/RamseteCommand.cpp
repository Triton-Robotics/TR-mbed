//
// Created by ben332004 on 3/13/23.
//

#include "RamseteCommand.h"

RamseteCommand::RamseteCommand(Pose2D startPose, Pose2D endPose, double durationSeconds, Chassis* chassis): controller(),
startPose(startPose), endPose(endPose) {
    this->startPose = startPose;
    this->endPose = endPose;
    this->durationSeconds = durationSeconds;
    this->chassis = chassis;
}

void RamseteCommand::initialize() {
    prevTime = -1;
    startTime = us_ticker_read() / 1000000.0;
}

void RamseteCommand::execute() {
    double curTime = us_ticker_read() / 1000000.0;
    double dt = curTime - prevTime;
    if (prevTime == -1) {
        prevTime = curTime;
        return;
    }

    Pose2D curPose = chassis->getPose();
    Pose2D desiredPose = endPose.interpolate(startPose, (curTime - startTime) / durationSeconds);

    ChassisSpeeds desiredSpeeds = controller.calculate(
            curPose,
            desiredPose,
            10 * (endPose.x - curPose.x),
            0
    );

    printf("speeds: %i %i %i\n", (int) desiredSpeeds.x, (int) desiredSpeeds.y, (int) desiredSpeeds.rotation);

    chassis->driveFieldRelative(desiredSpeeds);

    prevTime = curTime;
}

bool RamseteCommand::isFinished() {
    return false;
//    double curTime = us_ticker_read() / 1000000.0;
//    return (curTime - startTime) >= durationSeconds;
}