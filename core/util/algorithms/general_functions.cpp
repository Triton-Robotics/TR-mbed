#include "general_functions.h"

float calculateDeltaYaw(float curr_yaw, float des_yaw)
{
    float deltaYaw = des_yaw - curr_yaw;
    while (abs(deltaYaw) > 180)
    {
        if (deltaYaw > 0)
            deltaYaw -= 360;
        else
            deltaYaw += 360;
    }
    return deltaYaw;
}

float capAngle(float curr_angle)
{
    if (abs(curr_angle) > 180.0)
    {
        if (curr_angle > 0)
            curr_angle -= 360.0;
        else
            curr_angle += 360.0;
    }
    return curr_angle;
}