#include "math_utils.h"

float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

float wrap_angle(float angle) {
    while (angle > 3.14159265359f) angle -= 6.28318530718f;
    while (angle < -3.14159265359f) angle += 6.28318530718f;
    return angle;
}

float deadzone(float val, float threshold) {
    if (val > threshold) return val - threshold;
    if (val < -threshold) return val + threshold;
    return 0.0f;
}

float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
