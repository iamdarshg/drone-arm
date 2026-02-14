#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t gpio_pin;
    uint32_t frequency;
    uint16_t duty_cycle;
} pwm_config_t;

void pwm_init(uint8_t slice, pwm_config_t *config);
void pwm_set_duty(uint8_t slice, uint16_t duty);
uint16_t pwm_get_duty(uint8_t slice);
void pwm_set_frequency(uint8_t slice, uint32_t freq);
void pwm_enable(uint8_t slice);
void pwm_disable(uint8_t slice);
