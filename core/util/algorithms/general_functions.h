#include <zephyr/kernel.h>
#include <stdlib.h>

#define PI 3.14159265

float calculateDeltaYaw(float curr_yaw, float des_yaw);

float capAngle(float curr_angle);

float degreesToRadians(float degrees);

static inline uint64_t now_us(); // Replacement for us_ticker
