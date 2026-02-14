#include "filter.h"

void lowpass_init(lowpass_filter_t *f, float alpha) { f->alpha = alpha; f->value = 0.0f; }
float lowpass_update(lowpass_filter_t *f, float input) {
    f->value = f->alpha * input + (1.0f - f->alpha) * f->value;
    return f->value;
}

void biquad_init(biquad_filter_t *f, float b0, float b1, float b2, float a1, float a2) {
    f->a0 = b0; f->a1 = b1; f->a2 = b2; f->b1 = a1; f->b2 = a2;
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}
float biquad_update(biquad_filter_t *f, float input) {
    float output = f->a0 * input + f->a1 * f->x1 + f->a2 * f->x2 - f->b1 * f->y1 - f->b2 * f->y2;
    f->x2 = f->x1; f->x1 = input; f->y2 = f->y1; f->y1 = output;
    return output;
}
