#include "kernel/assert.h"
#include <stdint.h>

#include "drivers/clock.h"
#include "drivers/clock_internal.h"
#include "hal/platform.h"

enum {
    XOSC_CTRL = 0x00u,
    XOSC_STATUS = 0x04u,
    XOSC_STARTUP = 0x0Cu,
    PLL_CS = 0x00u,
    PLL_PWR = 0x04u,
    PLL_FBDIV_INT = 0x08u,
    PLL_PRIM = 0x0Cu,
    CLK_REF_CTRL = 0x30u,
    CLK_SYS_CTRL = 0x3Cu,
    CLK_PERI_CTRL = 0x48u,
    XOSC_ENABLE_VALUE = 0x00FABAA0u,
    CLOCK_TIMEOUT = 200000u,
};
static const uint32_t PLL_LOCK_BIT = UINT32_C(0x80000000);
static const uint32_t XOSC_STABLE_BIT = UINT32_C(0x80000000);

static uint32_t g_ref_hz;
static uint32_t g_sys_hz;
static uint32_t g_peri_hz;

static void wait_for_bit(uint32_t addr, uint32_t mask) {
    uint32_t i;
    for (i = 0u; i < CLOCK_TIMEOUT; ++i) {
        if ((REG_RO(addr) & mask) != 0u) {
            return;
        }
    }
    ASSERT(i < CLOCK_TIMEOUT);
}

void clock_init(void) {
    REG_RW(XOSC_BASE + XOSC_STARTUP) = 47u;
    REG_RW(XOSC_BASE + XOSC_CTRL) = XOSC_ENABLE_VALUE;
    wait_for_bit(XOSC_BASE + XOSC_STATUS, XOSC_STABLE_BIT);

    REG_RW(PLL_SYS_BASE + PLL_PWR) = 0u;
    REG_RW(PLL_SYS_BASE + PLL_CS) = 1u;
    REG_RW(PLL_SYS_BASE + PLL_FBDIV_INT) = 125u;
    REG_RW(PLL_SYS_BASE + PLL_PRIM) = (5u << 16) | 1u;
    wait_for_bit(PLL_SYS_BASE + PLL_CS, PLL_LOCK_BIT);

    REG_RW(CLOCKS_BASE + CLK_REF_CTRL) = 2u;
    REG_RW(CLOCKS_BASE + CLK_SYS_CTRL) = 1u;
    REG_RW(CLOCKS_BASE + CLK_PERI_CTRL) = 1u;

    g_ref_hz = 12000000u;
    g_sys_hz = 150000000u;
    g_peri_hz = 150000000u;
}

uint32_t clock_get_hz(clock_id_t id) {
    ASSERT(id <= CLOCK_ID_PERI);
    if (id == CLOCK_ID_REF) {
        return g_ref_hz;
    }
    if (id == CLOCK_ID_SYS) {
        return g_sys_hz;
    }
    return g_peri_hz;
}

/* ---------- internal state-tracking API (testable without hardware) ---------- */

#define CLOCK_STATE_MAGIC 0xC10CC10Cu

typedef struct {
    uint32_t magic;
    clock_config_t cfg;
} clock_state_t;

static clock_state_t g_clock_state;

void clock_state_init(uint32_t sys_hz, uint32_t vreg_mv) {
    ASSERT(sys_hz > 0u);
    ASSERT(vreg_mv > 0u);
    g_clock_state.cfg.sys_clk_hz = sys_hz;
    g_clock_state.cfg.vreg_voltage_mv = vreg_mv;
    g_clock_state.magic = CLOCK_STATE_MAGIC;
}

bool clock_is_valid(void) {
    return g_clock_state.magic == CLOCK_STATE_MAGIC;
}

const clock_config_t *clock_get_config(void) {
    if (g_clock_state.magic != CLOCK_STATE_MAGIC) {
        return NULL;
    }
    return &g_clock_state.cfg;
}

void clock_set_config(const clock_config_t *cfg) {
    ASSERT(cfg != NULL);
    ASSERT(cfg->sys_clk_hz > 0u);
    ASSERT(cfg->vreg_voltage_mv > 0u);
    g_clock_state.cfg = *cfg;
    g_clock_state.magic = CLOCK_STATE_MAGIC;
}
