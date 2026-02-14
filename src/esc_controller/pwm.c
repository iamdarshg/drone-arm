#include "pwm.h"

void pwm_init(uint8_t channel, uint32_t freq_hz) { (void)channel; (void)freq_hz; }
void pwm_set_duty(uint8_t channel, float duty) { (void)channel; (void)duty; }
float pwm_get_duty(uint8_t channel) { (void)channel; return 0.0f; }
void pwm_enable(uint8_t channel) { (void)channel; }
void pwm_disable(uint8_t channel) { (void)channel; }
