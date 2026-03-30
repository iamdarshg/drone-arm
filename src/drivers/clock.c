#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "drivers/clock.h"
#include "drivers/clock_internal.h"
#include "hal/platform.h"

/*
 * Register offsets within XOSC, PLL, and CLOCKS blocks.
 * (RP2350 TRM §2.16 – XOSC, §2.18 – PLLs, §2.15 – CLOCKS)
 */
enum {
    /* XOSC */
    XOSC_CTRL      = 0x00u,
    XOSC_STATUS    = 0x04u,
    XOSC_STARTUP   = 0x0Cu,
    /* PLL – shared by PLL_SYS and PLL_USB */
    PLL_CS         = 0x00u,   /* [5:0]=REFDIV, [31]=LOCK  */
    PLL_PWR        = 0x04u,   /* [0]=PD [2]=DSMPD [3]=POSTDIVPD [5]=VCOPD */
    PLL_FBDIV_INT  = 0x08u,
    PLL_PRIM       = 0x0Cu,   /* [18:16]=POSTDIV1  [14:12]=POSTDIV2       */
    /* CLOCKS */
    CLK_REF_CTRL   = 0x30u,   /* [1:0]=SRC (2=XOSC) */
    CLK_SYS_CTRL   = 0x3Cu,   /* [0]=SRC (0=CLK_REF, 1=aux)  [7:5]=AUXSRC */
    CLK_PERI_CTRL  = 0x48u,   /* [11]=ENABLE [7:5]=AUXSRC (0=CLK_SYS) */
    /* Magic / constants */
    XOSC_ENABLE_VALUE = 0x00FABAA0u,
    CLOCK_TIMEOUT     = 200000u,
    /* PLL_PWR bits */
    PLL_PWR_PD        = (1u << 0),
    PLL_PWR_DSMPD     = (1u << 2),  /* 1 = DSM off (integer mode) */
    PLL_PWR_POSTDIVPD = (1u << 3),
    PLL_PWR_VCOPD     = (1u << 5),
};

static const uint32_t PLL_LOCK_BIT   = UINT32_C(1) << 31;
static const uint32_t XOSC_STABLE_BIT = UINT32_C(1) << 31;
/* CLK_PERI_CTRL: ENABLE bit and AUXSRC=CLK_SYS (0) */
static const uint32_t CLK_PERI_ENABLE = UINT32_C(1) << 11;

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

static void pll_sys_init(void) {
    /*
     * PLL_SYS init sequence (RP2350 TRM §2.18.2):
     * Target: 12 MHz XOSC × 125 FBDIV = 1500 MHz VCO
     *         ÷ POSTDIV1=5 × POSTDIV2=2 → 150 MHz output.
     *
     * Step 1: Release PLL_SYS from reset (RESETS_CLR atomic write).
     * Step 2: Set REFDIV = 1.
     * Step 3: Set FBDIV = 125.
     * Step 4: Power on PLL and VCO (clear PD and VCOPD); keep DSMPD=1
     *         (integer-only mode, no Sigma-Delta Modulator needed) and
     *         POSTDIVPD=1 (post-dividers still gated).
     * Step 5: Wait for VCO lock.
     * Step 6: Programme post-dividers.
     * Step 7: Enable post-dividers (clear POSTDIVPD).
     */
    REG_CLR(RESETS_BASE + RESETS_RESET, RESETS_BIT_PLL_SYS);
    wait_for_bit(RESETS_BASE + RESETS_RESET_DONE, RESETS_BIT_PLL_SYS);

    /* REFDIV = 1 */
    REG_RW(PLL_SYS_BASE + PLL_CS) = 1u;
    /* FBDIV = 125  →  VCO = 12 MHz × 125 = 1500 MHz */
    REG_RW(PLL_SYS_BASE + PLL_FBDIV_INT) = 125u;
    /* Power on PLL + VCO; keep DSM and post-dividers gated for now */
    REG_RW(PLL_SYS_BASE + PLL_PWR) = PLL_PWR_DSMPD | PLL_PWR_POSTDIVPD;
    /* Wait for PLL to lock */
    wait_for_bit(PLL_SYS_BASE + PLL_CS, PLL_LOCK_BIT);
    /* POSTDIV1 = 5, POSTDIV2 = 2  →  output = 1500 / (5×2) = 150 MHz */
    REG_RW(PLL_SYS_BASE + PLL_PRIM) = (5u << 16) | (2u << 12);
    /* Enable post-dividers */
    REG_CLR(PLL_SYS_BASE + PLL_PWR, PLL_PWR_POSTDIVPD);
}

void clock_init(void) {
    /*
     * Full clock init sequence (RP2350 TRM §2.15):
     * 1. Start XOSC (12 MHz crystal).
     * 2. Switch CLK_REF to XOSC (away from default ROSC).
     * 3. Bring up PLL_SYS → 150 MHz.
     * 4. Switch CLK_SYS auxiliary source to PLL_SYS then switch CLK_SYS
     *    main mux to the auxiliary path (SRC=1).
     * 5. Enable CLK_PERI from CLK_SYS.
     */

    /* 1 – Enable XOSC */
    REG_RW(XOSC_BASE + XOSC_STARTUP) = 47u; /* ~1 ms @ 12 MHz: ceil(1ms/(256/12MHz)) */
    REG_RW(XOSC_BASE + XOSC_CTRL)    = XOSC_ENABLE_VALUE;
    wait_for_bit(XOSC_BASE + XOSC_STATUS, XOSC_STABLE_BIT);

    /* 2 – Switch CLK_REF to XOSC (SRC=2) */
    REG_RW(CLOCKS_BASE + CLK_REF_CTRL) = 2u;

    /* 3 – Init PLL_SYS → 150 MHz */
    pll_sys_init();

    /* 4 – CLK_SYS: set aux source to PLL_SYS (AUXSRC=0), then select aux */
    REG_RW(CLOCKS_BASE + CLK_SYS_CTRL) = 0u;   /* AUXSRC = PLL_SYS (bits[7:5]=0) */
    __dsb();
    REG_SET(CLOCKS_BASE + CLK_SYS_CTRL, 1u);   /* SRC bit 0 = 1 → select aux */

    /* 5 – CLK_PERI: enable, source = CLK_SYS (AUXSRC=0) */
    REG_RW(CLOCKS_BASE + CLK_PERI_CTRL) = CLK_PERI_ENABLE;

    g_ref_hz  = 12000000u;
    g_sys_hz  = 150000000u;
    g_peri_hz = 150000000u;

    clock_state_init(g_sys_hz, 1100u); /* 1.1 V VREG for 150 MHz */
}

uint32_t clock_get_hz(clock_id_t id) {
    ASSERT(id <= CLOCK_ID_PERI);
    if (id == CLOCK_ID_REF) { return g_ref_hz;  }
    if (id == CLOCK_ID_SYS) { return g_sys_hz;  }
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
