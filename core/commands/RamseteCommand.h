//
// Created by ben332004 on 3/13/23.
//

#ifndef TR_EMBEDDED_RAMSETECOMMAND_H
#define TR_EMBEDDED_RAMSETECOMMAND_H

#include <algorithms/RamseteController.h>
#include <Chassis.h>


/**
 * The RamseteCommand class represents a command for the robot to follow a specific path using Ramsete control. It
 * supports starting, running, and stopping the command.
 */
class RamseteCommand {
public:
    /**
     * Creates a RamseteCommand to drive the robot from one position to another in a specified time duration
     * @param startPose The start position and orientation as a Pose2D
     * @param endPose The end position and orientation as a Pose2D
     * @param durationSeconds The duration, in seconds, of the command
     * @param chassis A pointer to the robot's Chassis object
     */
    RamseteCommand(Pose2D startPose, Pose2D endPose, double durationSeconds, Chassis* chassis);

    /**
     * Initializes the command; must be run before the command can be run
     */
    void initialize();

    /**
     * Executes (runs) the command. This method should be called every loop cycle that the command is running.
     */
    void execute();

    /**
     * Ends the command. Called after the command finishes
     */
    void end();

    /**
     * Determines whether the command is finished running (i.e. the specified amount of time has passed)
     * @return true if the command is finished, false otherwise
     */
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
