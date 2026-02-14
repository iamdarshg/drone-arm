#include <stdint.h>

void pwm_init(uint8_t channel, uint32_t freq_hz);
void pwm_set_duty(uint8_t channel, float duty);
float pwm_get_duty(uint8_t channel);
void pwm_enable(uint8_t channel);
void pwm_disable(uint8_t channel);
