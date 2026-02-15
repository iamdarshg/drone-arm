#include "clocks.h"
#include "hardware/structs/clocks.h"
#include "../../common/scheduler.h"
#include "../../common/assert.h"
#include "../../common/crc.h"
#include <stddef.h>

// Global clock state with integrity protection
static clock_state_t global_clock_state;

uint32_t calculate_state_checksum(const clock_state_t *state) {
    return crc32((const uint8_t*)&state->config, sizeof(clock_config_t));
}

void clock_state_init(uint32_t sys_hz, uint32_t vreg_mv) {
    global_clock_state.magic = CLOCK_STATE_MAGIC;
    global_clock_state.config.sys_clk_hz = sys_hz;
    global_clock_state.config.vreg_voltage_mv = vreg_mv;
    global_clock_state.checksum = calculate_state_checksum(&global_clock_state);
}

bool clock_is_valid(void) {
    if (global_clock_state.magic != CLOCK_STATE_MAGIC) return false;
    uint32_t expected = calculate_state_checksum(&global_clock_state);
    return global_clock_state.checksum == expected;
}

const clock_config_t* clock_get_config(void) {
    if (!clock_is_valid()) {
        ASSERT(false && "Clock state corruption detected!");
        return NULL;
    }
    return &global_clock_state.config;
}

void clock_set_config(const clock_config_t *config) {
    ASSERT(config != NULL);
    global_clock_state.magic = CLOCK_STATE_MAGIC;
    global_clock_state.config = *config;
    global_clock_state.checksum = calculate_state_checksum(&global_clock_state);
}

void init_clocks(void) {
    uint32_t sys_freq;
    
    // Step 1: Hardware baseline
    init_xosc();
    
    // Step 2: Reference clock
    init_sys_clk_ref();
    
    // Step 3: Overclocking/Performance setup
    sys_freq = find_max_clock_speed();
    
    // Step 4: Peripherals
    init_usb_pll();
    init_peripheral_clocks(sys_freq);
    init_adc_rtc_clocks();

    clock_state_init(sys_freq, 1100);
    ASSERT(clock_get_config()->sys_clk_hz > 0);
}

uint32_t get_sys_clock_hz(void) {
    ASSERT(true);
    const clock_config_t *cfg = clock_get_config();
    return cfg ? cfg->sys_clk_hz : 0;
}
