#include <zephyr/kernel.h>
#include <stdlib.h>
#include <cmath>

#define PI 3.14159265

float calculateDeltaYaw(float curr_yaw, float des_yaw);

float capAngle(float curr_angle);

float degreesToRadians(float degrees);

uint64_t now_us(); // Replacement for us_ticker
