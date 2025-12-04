#include "RobotState.h"

namespace TR
{

    // #ifdef USE_IMU
    //     BNO055_ANGULAR_POSITION_typedef imuAngles;
    // #endif

    // Variables for burst fire
    // unsigned long timeSure;
    // unsigned long prevTimeSure;
    // bool shoot = false;
    // int shootTargetPosition = 36 * 8190;
    // bool shootReady = false;
    // int remoteTimer = 0;

    // // Remote control variables
    // float scalar = 1;
    // float jx = 0; // -1 to 1
    // float jy = 0; // -1 to 1
    // // Pitch, Yaw
    // float jpitch = 0; // -1 to 1
    // float jyaw = 0;   // -1 to 1
    // float myaw = 0;
    // float mpitch = 0;
    // int pitchVelo = 0;
    // // joystick tolerance
    // float tolerance = 0.05;
    // // Keyboard Driving
    // float mult = 0.7;
    // float omega_speed = 0;
    // float max_linear_vel = 0;

    // // drive and shooting mode
    // char drive = 'o'; // default o when using joystick
    // char shot = 'o';  // default o when using joystick
    
    // GENERAL VARIABLES
    // TODO: make a general IMU class that other imu classes inherit
    // IMU
    BNO055_ANGULAR_POSITION_typedef imuAnglesLocal;

    // Chassis control variables
    float chassis_x = 0;
    float chassis_y = 0;

    // Turret controls variables
    float yaw_desired_angle = 0;
    float pitch_desired_angle = 0;
    // float yaw_current_angle = 0;
    // float pitch_current_angle = 0;

    // // ref variables
    // uint16_t chassis_power_limit;

    // timers
    unsigned long timeStart;
    unsigned long timeStartCV;
    unsigned long timeStartRef;
    unsigned long timeStartImu;
    // TODO this used to be = us_ticker_read() might cause problems
    unsigned long loopTimer = us_ticker_read();
    unsigned long loopTimerCV = loopTimer;
    unsigned long loopTimerRef = loopTimer;
    unsigned long loopTimerImu = loopTimer;
    float elapsedms = 0.0; // TODO see if this is still needed

    // int readResult = 0;
    // bool cv_enabled = false;
    // char cv_shoot_status = 0;
}