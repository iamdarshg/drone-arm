#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t baud_rate;
} i2c_config_t;

void i2c_init(uint8_t i2c_id, i2c_config_t *config);
bool i2c_write(uint8_t i2c_id, uint8_t addr, const uint8_t *data, uint32_t len);
bool i2c_read(uint8_t i2c_id, uint8_t addr, uint8_t *data, uint32_t len);
bool i2c_write_reg(uint8_t i2c_id, uint8_t addr, uint8_t reg, uint8_t value);
uint8_t i2c_read_reg(uint8_t i2c_id, uint8_t addr, uint8_t reg);
int16_t i2c_scan(uint8_t i2c_id);
