#include "kernel/assert.h"

#include "board.h"
#include "board_config.h"
#include "drivers/clock.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "drivers/spi.h"
#include "drivers/uart.h"
#include "hal/platform.h"
#include "system/multicore.h"

/*
 * Core1 stack – placed in Scratch-Y SRAM (4 KB, RP2350 TRM §2.2.2).
 * Stack grows downward; we pass the top address to the ROM launch sequence.
 */
#define CORE1_STACK_WORDS 256u   /* 1 KB */
static uint32_t g_core1_stack[CORE1_STACK_WORDS];

/* Defined in main.c – Core1's application entry point. */
extern void core1_main(void);

/*
 * Release a set of peripherals from the RESETS block and wait until they
 * confirm they have come out of reset (RESET_DONE).
 *
 * The RESETS peripheral supports atomic alias writes (CLR at +0x3000).
 * Writing 0 to a RESET bit releases the peripheral; RESET_DONE reads 1
 * when the peripheral is ready (RP2350 TRM §4.2).
 */
static void board_release_resets(uint32_t mask) {
    uint32_t i;
    ASSERT(mask != 0u);
    REG_CLR(RESETS_BASE + RESETS_RESET, mask);
    for (i = 0u; i < 200000u; ++i) {
        if ((REG_RO(RESETS_BASE + RESETS_RESET_DONE) & mask) == mask) {
            return;
        }
    }
    ASSERT(i < 200000u);
}

static void board_config_pins(void) {
    gpio_set_function(BOARD_LED_PIN,      5u); /* SIO */
    gpio_set_dir(BOARD_LED_PIN, true);

    gpio_set_function(BOARD_UART0_TX_PIN, 2u); /* UART */
    gpio_set_function(BOARD_UART0_RX_PIN, 2u);

    gpio_set_function(BOARD_I2C0_SDA_PIN, 3u); /* I2C */
    gpio_set_function(BOARD_I2C0_SCL_PIN, 3u);

    gpio_set_function(BOARD_SPI0_TX_PIN,  1u); /* SPI */
    gpio_set_function(BOARD_SPI0_RX_PIN,  1u);
    gpio_set_function(BOARD_SPI0_SCK_PIN, 1u);
    gpio_set_function(BOARD_SPI0_CS_PIN,  1u);
}

void board_init(void) {
    uint32_t *core1_sp;
    bool launched;

    /* Step 1 – Bring up clocks (XOSC → PLL_SYS → 150 MHz).
     *           clock_init() releases PLL_SYS from RESETS internally. */
    clock_init();

    /* Step 2 – Release all required peripherals from reset before any
     *           driver touches their registers. */
    board_release_resets(
        RESETS_BIT_IO_BANK0   |
        RESETS_BIT_PADS_BANK0 |
        RESETS_BIT_UART0      |
        RESETS_BIT_I2C0       |
        RESETS_BIT_SPI0       |
        RESETS_BIT_DMA        |
        RESETS_BIT_PIO0       |
        RESETS_BIT_PIO1       |
        RESETS_BIT_PIO2
    );

    /* Step 3 – GPIO: set all pins to SIO function with safe pad settings. */
    gpio_init();
    board_config_pins();

    /* Step 4 – Peripheral drivers. */
    uart_init(115200u);
    i2c_init(100000u);
    spi_init(1000000u);

    /* Step 5 – Announce boot success before launching Core1. */
    (void)uart_puts("Board Init Complete\r\n");

    /* Step 6 – Launch Core1 with proper RP2350 ROM FIFO boot protocol. */
    core1_sp = g_core1_stack + CORE1_STACK_WORDS;
    launched = multicore_launch_core1(core1_main, core1_sp);
    ASSERT(launched);
}
