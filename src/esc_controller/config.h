#include <stdint.h>

typedef struct {
    uint16_t motor_poles;
    uint16_t pwm_frequency;
    uint16_t control_frequency;
    float current_limit;
    float voltage_limit;
    float kp, ki, kd;
} esc_config_t;

esc_config_t esc_get_config(uint8_t id);
void esc_set_config(uint8_t id, esc_config_t config);
void esc_load_defaults(uint8_t id);
