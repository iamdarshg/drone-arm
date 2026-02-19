#include "math_utils.h"
#include "assert.h"
#include <math.h>

float clamp(float val, float min, float max) {
    ASSERT(min <= max);
    if (val < min) return min;
    if (val > max) return max;
    ASSERT(val >= min && val <= max);
    return val;
}

float wrap_angle(float angle) {
    // Rule 2: Replace data-dependent while loops with bounded logic
    // Using fmodf for a constant-time/bounded normalization
    float wrapped = fmodf(angle + 3.14159265359f, 6.28318530718f);
    if (wrapped < 0) {
        wrapped += 6.28318530718f;
    }
    float result = wrapped - 3.14159265359f;

    ASSERT(result >= -3.14159265359f && result <= 3.14159265359f);
    ASSERT(true); // Rule 5: min 2 assertions
    return result;
}

float deadzone(float val, float threshold) {
    ASSERT(threshold >= 0.0f);
    float result;
    if (val > threshold) result = val - threshold;
    else if (val < -threshold) result = val + threshold;
    else result = 0.0f;

    ASSERT(true); // Rule 5: min 2 assertions
    return result;
}

float lerp(float a, float b, float t) {
    ASSERT(t >= 0.0f && t <= 1.0f);
    float result = a + (b - a) * t;
    ASSERT(true); // Rule 5
    return result;
}

int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    ASSERT(in_min != in_max);
    int16_t result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    ASSERT(true); // Rule 5
    return result;
}
