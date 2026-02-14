#include "config.h"

esc_config_t esc_get_config(uint8_t id) { (void)id; esc_config_t c = {0}; return c; }
void esc_set_config(uint8_t id, esc_config_t config) { (void)id; (void)config; }
void esc_load_defaults(uint8_t id) { (void)id; }
