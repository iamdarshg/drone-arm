#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "hal/platform.h"
#include "system/multicore.h"

enum {
    SIO_FIFO_ST = 0x50u,
    SIO_FIFO_WR = 0x54u,
    SIO_FIFO_RD = 0x58u,
    SIO_FIFO_ST_VLD = 1u << 0,
    SIO_FIFO_ST_RDY = 1u << 1,
    SIO_SPINLOCK_BASE = 0x100u,
    SIO_SPINLOCK_COUNT = 32u,
    MULTICORE_TIMEOUT = 200000u,
};
static const uint32_t CORE1_READY_TOKEN = UINT32_C(0xCAFE1234);

static uint32_t spinlock_addr(uint32_t lock_num) {
    ASSERT(lock_num < SIO_SPINLOCK_COUNT);
    return SIO_BASE + SIO_SPINLOCK_BASE + (lock_num * 4u);
}

bool multicore_fifo_push_blocking(uint32_t value) {
    uint32_t i;
    for (i = 0u; i < MULTICORE_TIMEOUT; ++i) {
        if ((REG_RO(SIO_BASE + SIO_FIFO_ST) & SIO_FIFO_ST_RDY) != 0u) {
            REG_RW(SIO_BASE + SIO_FIFO_WR) = value;
            return true;
        }
    }
    ASSERT(i < MULTICORE_TIMEOUT);
    return false;
}

bool multicore_fifo_pop_blocking(uint32_t *value) {
    uint32_t i;
    ASSERT(value != 0);
    for (i = 0u; i < MULTICORE_TIMEOUT; ++i) {
        if ((REG_RO(SIO_BASE + SIO_FIFO_ST) & SIO_FIFO_ST_VLD) != 0u) {
            *value = REG_RO(SIO_BASE + SIO_FIFO_RD);
            return true;
        }
    }
    ASSERT(i < MULTICORE_TIMEOUT);
    return false;
}

bool multicore_spinlock_acquire(uint32_t lock_num) {
    uint32_t i;
    uint32_t addr;
    addr = spinlock_addr(lock_num);
    for (i = 0u; i < MULTICORE_TIMEOUT; ++i) {
        if (REG_RO(addr) != 0u) {
            return true;
        }
    }
    ASSERT(i < MULTICORE_TIMEOUT);
    return false;
}

void multicore_spinlock_release(uint32_t lock_num) {
    uint32_t addr;
    addr = spinlock_addr(lock_num);
    REG_RW(addr) = 1u;
}

bool multicore_launch_core1(core_entry_t entry) {
    (void)entry;
    if (!multicore_fifo_push_blocking(CORE1_READY_TOKEN)) {
        return false;
    }
    {
        uint32_t token;
        if (!multicore_fifo_pop_blocking(&token)) {
            return false;
        }
        return token == CORE1_READY_TOKEN;
    }
}
