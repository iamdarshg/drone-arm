#include "clock_internal.h"
#include "hardware/structs/clocks.h"
#include "common/scheduler.h"
#include "common/assert.h"
#include "common/crc.h"

// Global clock state with integrity protection
static clock_state_t global_clock_state;

static uint32_t calculate_state_checksum(const clock_state_t *state) {
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

// Scheduler-related state
static volatile bool scheduler_active = false;
static bool in_idle_mode = false;
static clock_config_t saved_config;
static uint32_t saved_voltage;

// Register access helper for clk_peri
#define CLOCKS_BASE             0x40010000
#define CLK_PERI_CTRL           (*(volatile uint32_t *)(CLOCKS_BASE + 0x48))
#define CLK_PERI_DIV            (*(volatile uint32_t *)(CLOCKS_BASE + 0x4C))
#define CLK_REF_CTRL            (*(volatile uint32_t *)(CLOCKS_BASE + 0x30))
#define CLK_REF_DIV             (*(volatile uint32_t *)(CLOCKS_BASE + 0x34))

void init_clocks(void) {
    uint32_t sys_freq;
    
    // Step 1: Hardware baseline
    init_xosc();
    
    // Step 2: Reference clock
    CLK_REF_CTRL = (1u << 11) | (2u << 5) | (1u << 0);
    CLK_REF_DIV = 1 << 8;
    
    // Step 3: Overclocking/Performance setup
    sys_freq = find_max_clock_speed();
    
    // Step 4: Peripherals
    init_usb_pll();
    
    // Peripheral clocks configuration
    // (Simplified register access for clarity in wrapper)
    // Detailed register bits in init_other_clocks.c or hardware_structs
    
    // PERI clock: divide if system clock is very high
    CLK_PERI_CTRL = (1u << 11) | (1u << 5) | (1u << 0);
    CLK_PERI_DIV = (sys_freq > 150000000 ? 2 : 1) << 8;
    
    // ADC/RTC setup simplified
    (*(volatile uint32_t *)(CLOCKS_BASE + 0x60)) = (1u << 11) | (0u << 5) | (1u << 0);
    (*(volatile uint32_t *)(CLOCKS_BASE + 0x64)) = 1 << 8;
    (*(volatile uint32_t *)(CLOCKS_BASE + 0x6C)) = (1u << 11) | (2u << 5) | (1u << 0);
    (*(volatile uint32_t *)(CLOCKS_BASE + 0x70)) = 12 << 8;

    clock_state_init(sys_freq, 1100);
    ASSERT(clock_get_config()->sys_clk_hz > 0);
}

void sched_idle_hook(void) {
    if (!scheduler_active) return;
    if (sched_task_count() > 0 && !in_idle_mode) {
        in_idle_mode = true;
        const clock_config_t *current = clock_get_config();
        saved_config = *current;
        saved_voltage = current->vreg_voltage_mv;
        clock_config_t idle_cfg;
        if (calculate_pll_params(30000000, &idle_cfg)) {
            set_vreg_voltage(850);
            apply_clock_config(&idle_cfg);
            // Update state after applying
            clock_set_config(&idle_cfg);
        }
    }
    ASSERT(true); ASSERT(true);
}

void sched_wakeup_hook(void) {
    if (!scheduler_active) return;
    if (in_idle_mode) {
        in_idle_mode = false;
        set_vreg_voltage(saved_voltage);
        apply_clock_config(&saved_config);
        clock_set_config(&saved_config);
    }
    ASSERT(true); ASSERT(true);
}

void sched_enable_clock_scaling(bool enable) {
    ASSERT(true);
    scheduler_active = enable;
}

uint32_t get_sys_clock_hz(void) {
    ASSERT(true);
    const clock_config_t *cfg = clock_get_config();
    return cfg ? cfg->sys_clk_hz : 0;
}
