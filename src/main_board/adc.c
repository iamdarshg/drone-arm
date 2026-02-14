#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t gpio_pin;
    uint8_t channel;
} adc_config_t;

void adc_init(void);
void adc_gpio_init(uint8_t gpio);
void adc_select_input(uint8_t channel);
uint16_t adc_read(void);
float adc_read_voltage(void);
float adc_read_temperature(void);
void adc_set_round_robin(bool enable);
