#include "general_functions.h"
#include <cmath>

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
    if (fabs(curr_angle) > 180.0)
    {
        if (curr_angle > 0)
            curr_angle -= 360.0;
        else
            curr_angle += 360.0;
    }
    return curr_angle;
}

static inline uint64_t now_us()
{
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

