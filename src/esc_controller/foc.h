#include <stdint.h>

void foc_init(void);
void foc_set_dq(uint8_t id, float d, float q);
void foc_get_dq(uint8_t id, float *d, float *q);
void foc_sine_init(uint8_t id, float electrical_angle);
float foc_sine(uint8_t id);
