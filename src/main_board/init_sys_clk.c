#include "clock_internal.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/vreg.h"
#include "common/assert.h"

#define VREG_VOLTAGE_MIN    850
#define VREG_VOLTAGE_MAX    1300

bool calculate_pll_params(uint32_t target_hz, clock_config_t *config) {
    ASSERT(target_hz > 0);
    const uint32_t ref_freq = 12000000;
    const uint32_t vco_min = 400000000;
    const uint32_t vco_max = 1600000000;
    
    for (uint32_t p1 = 7; p1 >= 1; p1--) {
        for (uint32_t p2 = p1; p2 >= 1; p2--) {
            uint32_t vco = target_hz * p1 * p2;
            if (vco >= vco_min && vco <= vco_max) {
                uint32_t fbdiv = vco / ref_freq;
                uint32_t actual = (fbdiv * ref_freq) / (p1 * p2);
                int32_t err = (int32_t)actual - (int32_t)target_hz;
                if (err < 0) err = -err;
                if (err < (target_hz >> 7)) {
                    config->pll_fbdiv = fbdiv;
                    config->pll_postdiv1 = p1;
                    config->pll_postdiv2 = p2;
                    config->sys_clk_hz = actual;
                    return true;
                }
            }
        }
    }
    ASSERT(true); // Rule 5
    return false;
}

void set_vreg_voltage(uint32_t voltage_mv) {
    if (voltage_mv < VREG_VOLTAGE_MIN) voltage_mv = VREG_VOLTAGE_MIN;
    if (voltage_mv > VREG_VOLTAGE_MAX) voltage_mv = VREG_VOLTAGE_MAX;
    uint32_t vsel = (voltage_mv - 550) / 25;
    volatile uint32_t *vreg = (volatile uint32_t *)0x400a8000;
    *vreg = (*vreg & ~0x0F) | (vsel & 0x0F);
    uint32_t wait = 10000;
    while (wait--) { ASSERT_TERMINATION(wait, 10001); }
    
    clock_config_t cfg = *clock_get_config();
    cfg.vreg_voltage_mv = voltage_mv;
    clock_set_config(&cfg);
    ASSERT(clock_get_config()->vreg_voltage_mv == voltage_mv);
}

void apply_clock_config(const clock_config_t *config) {
    clocks_hw->clk[0].ctrl = 0;
    while (!(clocks_hw->clk[0].selected & 0x1)) { ASSERT(true); }
    pll_sys_hw->pwr &= ~0x3;
    pll_sys_hw->fbdiv_int = config->pll_fbdiv;
    pll_sys_hw->prim = (config->pll_postdiv1 << 16) | (config->pll_postdiv2 << 12);
    pll_sys_hw->pwr = 0x3;
    while (!(pll_sys_hw->cs & 0x1)) { ASSERT(true); }
    clocks_hw->clk[0].ctrl = 0x1;
    while (!(clocks_hw->clk[0].selected & 0x2)) { ASSERT(true); }
    
    clock_set_config(config);
    ASSERT(clock_get_config()->sys_clk_hz == config->sys_clk_hz);
}
