#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t miso_pin;
    uint8_t mosi_pin;
    uint8_t sck_pin;
    uint8_t cs_pin;
    uint32_t baud_rate;
} spi_config_t;

void spi_init(uint8_t spi_id, spi_config_t *config);
void spi_transfer(uint8_t spi_id, const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);
uint8_t spi_transfer_byte(uint8_t spi_id, uint8_t tx);
void spi_cs_select(uint8_t spi_id, uint8_t cs);
void spi_cs_deselect(uint8_t spi_id, uint8_t cs);
