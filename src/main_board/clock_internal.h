#ifndef CLOCK_INTERNAL_H
#define CLOCK_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>

// Clock configuration parameters
typedef struct {
    uint32_t sys_clk_hz;
    uint32_t pll_fbdiv;
    uint32_t pll_postdiv1;
    uint32_t pll_postdiv2;
    uint32_t vreg_voltage_mv;
} clock_config_t;

// Clock state structure with integrity protection
typedef struct {
    uint32_t magic;
    clock_config_t config;
    uint32_t checksum;
} clock_state_t;

#define CLOCK_STATE_MAGIC 0x434C4B53 // 'CLKS'

/**
 * @brief Initialize the clock state management system.
 * @param sys_hz Initial system clock frequency in Hz.
 * @param vreg_mv Initial VREG voltage in mV.
 */
void clock_state_init(uint32_t sys_hz, uint32_t vreg_mv);

/**
 * @brief Get the current clock configuration.
 * @return Pointer to the current clock configuration, or NULL if state is invalid.
 */
const clock_config_t* clock_get_config(void);

/**
 * @brief Update the clock configuration safely.
 * @param config New clock configuration to apply.
 */
void clock_set_config(const clock_config_t *config);

/**
 * @brief Verify the integrity of the clock state.
 * @return true if valid, false if corrupted.
 */
bool clock_is_valid(void);

// Function prototypes for modular clock components
bool calculate_pll_params(uint32_t target_hz, clock_config_t *config);
void set_vreg_voltage(uint32_t voltage_mv);
void apply_clock_config(const clock_config_t *config);

bool test_flash_stability(void);
bool test_adc_stability(void);
bool test_spi_stability(void);
bool test_i2c_stability(void);

uint32_t find_max_clock_speed(void);

void init_usb_pll(void);
void init_xosc(void);

#endif // CLOCK_INTERNAL_H
