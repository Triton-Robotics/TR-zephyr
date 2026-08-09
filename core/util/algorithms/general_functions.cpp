
#include "general_functions.h"


static inline uint64_t now_us()
{
    return k_ticks_to_us_floor64(k_uptime_ticks());
}