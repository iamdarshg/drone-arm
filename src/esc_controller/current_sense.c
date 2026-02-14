#include "current_sense.h"

void current_sense_init(uint8_t id) { (void)id; }
current_data_t current_read(uint8_t id) { (void)id; current_data_t c = {0}; return c; }
void current_calibrate(uint8_t id) { (void)id; }
