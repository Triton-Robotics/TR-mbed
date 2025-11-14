#ifndef TR_EMBEDDED_SENTRY_H
#define TR_EMBEDDED_SENTRY_H
#include "main.h"
#include "subsystems/ChassisSubsystem.h"
// Add Sentry constants here

//CONSTANTS
constexpr float LOWERBOUND = -35.0;
constexpr float UPPERBOUND = 40.0;

constexpr int OUTER_LOOP_DT_MS = 1;

constexpr float CHASSIS_FF_KICK = 0.065;

constexpr float pitch_zero_offset_ticks = 1500;

constexpr int NUM_BALLS_SHOT = 3;
constexpr int FLYWHEEL_VELO = 5500;

// Chassis Definitions
ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.22617); // radius is 9 in
DJIMotor yaw(4, CANHandler::CANBUS_1, GIMBLY,"Yeah");
DJIMotor pitch(7, CANHandler::CANBUS_2, GIMBLY,"Peach"); // right
DJIMotor indexer(7, CANHandler::CANBUS_2, C610,"Indexer");
DJIMotor RFLYWHEEL(1, CANHandler::CANBUS_2, M3508,"RightFly");
DJIMotor LFLYWHEEL(2, CANHandler::CANBUS_2, M3508,"LeftFly");

// robot specific sillies (gonna be deprecated)
int yawVelo = 0;
PID pitchCascade(1.5,0.0005,0.05);
PID sure(0.1,0,0.001);
ChassisSpeeds cs;

void init();
void chassis_executor();
void gimbal_executor();
void shoot_executor();
void jetson_executor();
void ref_thread();
void imu_thread();

#endif //TR_EMBEDDED_SENTRY_H
