#include <stdint.h>
#include <math.h>

float clamp(float val, float min, float max);
float wrap_angle(float angle);
float deadzone(float val, float threshold);
float lerp(float a, float b, float t);
int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
