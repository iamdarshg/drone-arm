#include "kernel/assert.h"

#include "board.h"
#include "board_config.h"
#include "drivers/clock.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "drivers/spi.h"
#include "drivers/uart.h"
#include "system/multicore.h"

static void board_config_pins(void) {
    gpio_set_function(BOARD_LED_PIN, 5u);
    gpio_set_dir(BOARD_LED_PIN, true);

    gpio_set_function(BOARD_UART0_TX_PIN, 2u);
    gpio_set_function(BOARD_UART0_RX_PIN, 2u);

    gpio_set_function(BOARD_I2C0_SDA_PIN, 3u);
    gpio_set_function(BOARD_I2C0_SCL_PIN, 3u);

    gpio_set_function(BOARD_SPI0_TX_PIN, 1u);
    gpio_set_function(BOARD_SPI0_RX_PIN, 1u);
    gpio_set_function(BOARD_SPI0_SCK_PIN, 1u);
    gpio_set_function(BOARD_SPI0_CS_PIN, 1u);
}

void board_init(void) {
    bool launched;
    clock_init();
    gpio_init();
    board_config_pins();
    uart_init(115200u);
    i2c_init(100000u);
    spi_init(1000000u);
    launched = multicore_launch_core1();
    ASSERT(launched);
}
