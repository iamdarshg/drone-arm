#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float voltage;
    float current;
    float rpm;
    int16_t temperature;
} esc_state_t;

void esc_init(uint8_t id);
void esc_set_target(uint8_t id, float duty_cycle);
esc_state_t esc_read_state(uint8_t id);
void esc_enable(uint8_t id);
void esc_disable(uint8_t id);
