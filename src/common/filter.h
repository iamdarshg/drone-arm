#include <stdint.h>

typedef struct {
    float alpha;
    float value;
} lowpass_filter_t;

typedef struct {
    float a0, a1, a2, b1, b2;
    float x1, x2, y1, y2;
} biquad_filter_t;

void lowpass_init(lowpass_filter_t *f, float alpha);
float lowpass_update(lowpass_filter_t *f, float input);

void biquad_init(biquad_filter_t *f, float b0, float b1, float b2, float a1, float a2);
float biquad_update(biquad_filter_t *f, float input);
