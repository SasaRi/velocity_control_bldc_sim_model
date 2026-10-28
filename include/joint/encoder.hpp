#pragma once

#include "joint/motor_params.hpp"

#include <cstdint>

namespace joint {

// Signed count difference between two single-turn readings, taking the
// shortest way around the wrap
inline int32_t position_difference_calculation_tcks(uint32_t now, uint32_t prev, uint32_t counts) {
    const int32_t range = static_cast<int32_t>(counts);  // 131072
    const int32_t half  = range / 2;                     // 65536

    int32_t delta = static_cast<int32_t>(now) - static_cast<int32_t>(prev);

    if (delta > half)  delta -= range;  // passed 0 going backwards (0 -> 131071)
    if (delta < -half) delta += range;  // passed 131071 going forwards (131071 -> 0)
    return delta;
}

inline double tcks_to_rad(int32_t counts, uint32_t counts_per_rev) {
    return static_cast<double>(counts) * (Two_Pi / static_cast<double>(counts_per_rev));
}

} // namespace joint
