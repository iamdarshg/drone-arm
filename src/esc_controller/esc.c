#include "esc.h"

void esc_init(uint8_t id) {
    (void)id;
}

void esc_set_target(uint8_t id, float duty_cycle) {
    (void)id;
    (void)duty_cycle;
}

esc_state_t esc_read_state(uint8_t id) {
    (void)id;
    esc_state_t state = {0};
    return state;
}

void esc_enable(uint8_t id) {
    (void)id;
}

void esc_disable(uint8_t id) {
    (void)id;
}
