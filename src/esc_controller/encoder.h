#include <stdint.h>

typedef struct {
    int32_t position;
    float velocity;
} encoder_data_t;

void encoder_init(uint8_t id);
encoder_data_t encoder_read(uint8_t id);
void encoder_reset(uint8_t id, int32_t position);
