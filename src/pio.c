#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "hal/platform.h"

enum {
    PIO_CTRL = 0x00u,
    PIO_FSTAT = 0x04u,
    PIO_FLEVEL = 0x0Cu,
    PIO_TXF0 = 0x10u,
    PIO_RXF0 = 0x20u,
    PIO_SM0_CLKDIV = 0xC8u,
    PIO_SM0_EXECCTRL = 0xCCu,
    PIO_SM0_SHIFTCTRL = 0xD0u,
    PIO_SM0_ADDR = 0xD4u,
    PIO_SM0_INSTR = 0xD8u,
    PIO_INSTR_MEM0 = 0x48u,
    PIO_SM_COUNT = 4u,
    PIO_INSTR_COUNT = 32u,
};

void pio_init(uint32_t pio_base) {
    uint32_t sm;
    ASSERT(pio_base == PIO0_BASE || pio_base == PIO1_BASE || pio_base == PIO2_BASE);
    ASSERT(REG_RO(pio_base + PIO_FSTAT) != 0xFFFFFFFFu); // Check peripheral presence

    REG_RW(pio_base + PIO_CTRL) = 0u;
    for (sm = 0u; sm < PIO_SM_COUNT; ++sm) {
        uint32_t off = sm * 0x18u;
        REG_RW(pio_base + PIO_SM0_CLKDIV + off) = 1u << 16;
        REG_RW(pio_base + PIO_SM0_EXECCTRL + off) = 0x1F000000u;
        REG_RW(pio_base + PIO_SM0_SHIFTCTRL + off) = 0x000C0000u;
    }
}

bool pio_load_program(uint32_t pio_base, const uint16_t *instr, uint32_t len, uint32_t origin) {
    uint32_t i;
    ASSERT(instr != 0);
    ASSERT(origin + len <= PIO_INSTR_COUNT);
    ASSERT(pio_base != 0u);
    for (i = 0u; i < len; ++i) {
        REG_RW(pio_base + PIO_INSTR_MEM0 + (origin + i) * 4u) = instr[i];
    }
    return true;
}

void pio_sm_set_enabled(uint32_t pio_base, uint32_t sm, bool enabled) {
    uint32_t val;
    ASSERT(sm < PIO_SM_COUNT);
    ASSERT(pio_base != 0u);
    val = REG_RO(pio_base + PIO_CTRL);
    if (enabled) {
        val |= (1u << sm);
    } else {
        val &= ~(1u << sm);
    }
    REG_RW(pio_base + PIO_CTRL) = val;
}
