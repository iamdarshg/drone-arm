#include "encoder.h"

void encoder_init(uint8_t id) { (void)id; }
encoder_data_t encoder_read(uint8_t id) { (void)id; encoder_data_t d = {0}; return d; }
void encoder_reset(uint8_t id, int32_t position) { (void)id; (void)position; }
