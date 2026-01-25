#include "general_functions.h"

int calculateDeltaYaw(int curr_yaw, int des_yaw)
{
    int deltaYaw = des_yaw - curr_yaw;
    if (abs(deltaYaw) > 180)
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