#include <stdint.h>

typedef struct {
    float ia, ib, ic;
    float power;
} current_data_t;

void current_sense_init(uint8_t id);
current_data_t current_read(uint8_t id);
void current_calibrate(uint8_t id);
