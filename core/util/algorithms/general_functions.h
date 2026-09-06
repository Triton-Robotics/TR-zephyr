#include <zephyr/kernel.h>
#include <stdlib.h>
#include <cmath>

#define PI 3.14159265

float calculateDeltaYaw(float curr_yaw, float des_yaw);

float capAngle(float curr_angle);

double degreesToRadians(double degrees);

double radiansToDegrees(double radians);


/**
* @brief Returns the current microsecond ticks
*/
uint64_t now_us(); 
