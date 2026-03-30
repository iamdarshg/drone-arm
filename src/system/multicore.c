#include "kernel/assert.h"
#include <stdbool.h>
#include <stdint.h>

#include "hal/platform.h"
#include "system/multicore.h"

enum {
    SIO_FIFO_ST          = 0x50u,
    SIO_FIFO_WR          = 0x54u,
    SIO_FIFO_RD          = 0x58u,
    SIO_FIFO_ST_VLD      = (1u << 0),  /* RX FIFO has data  */
    SIO_FIFO_ST_RDY      = (1u << 1),  /* TX FIFO has space */
    SIO_SPINLOCK_BASE    = 0x100u,
    SIO_SPINLOCK_COUNT   = 32u,
    MULTICORE_TIMEOUT    = 200000u,
    MULTICORE_CMD_COUNT  = 6u,
};

/* ---- spinlock helpers --------------------------------------------------- */

static uint32_t spinlock_addr(uint32_t lock_num) {
    ASSERT(lock_num < SIO_SPINLOCK_COUNT);
    return SIO_BASE + SIO_SPINLOCK_BASE + (lock_num * 4u);
}

bool multicore_fifo_push_blocking(uint32_t value) {
    uint32_t i;
    for (i = 0u; i < MULTICORE_TIMEOUT; ++i) {
        if ((REG_RO(SIO_BASE + SIO_FIFO_ST) & SIO_FIFO_ST_RDY) != 0u) {
            REG_RW(SIO_BASE + SIO_FIFO_WR) = value;
            __dsb();
            return true;
        }
    }
    ASSERT(i < MULTICORE_TIMEOUT);
    return false;
}

bool multicore_fifo_pop_blocking(uint32_t *value) {
    uint32_t i;
    ASSERT(value != NULL);
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
    uint32_t addr = spinlock_addr(lock_num);
    for (i = 0u; i < MULTICORE_TIMEOUT; ++i) {
        if (REG_RO(addr) != 0u) { return true; }
    }
    ASSERT(i < MULTICORE_TIMEOUT);
    return false;
}

void multicore_spinlock_release(uint32_t lock_num) {
    REG_RW(spinlock_addr(lock_num)) = 1u;
}

/* ---- Core1 launch -------------------------------------------------------- */

/*
 * Drain the SIO RX FIFO and send a SEV to wake Core1's WFE loop.
 * Called before each 0-word in the boot sequence.
 */
static void fifo_drain_and_wake(void) {
    while ((REG_RO(SIO_BASE + SIO_FIFO_ST) & SIO_FIFO_ST_VLD) != 0u) {
        (void)REG_RO(SIO_BASE + SIO_FIFO_RD);
    }
    __dsb();
    __sev();
}

/*
 * Push one word and wait for Core1 to echo it back.
 * Returns false on timeout; resets seq to 0 if the echo is wrong.
 */
static bool fifo_cmd_exchange(uint32_t cmd, uint32_t *seq) {
    uint32_t i;
    uint32_t response;
    ASSERT(seq != NULL);
    /* Push */
    i = 0u;
    while (i < MULTICORE_TIMEOUT) {
        if ((REG_RO(SIO_BASE + SIO_FIFO_ST) & SIO_FIFO_ST_RDY) != 0u) {
            REG_RW(SIO_BASE + SIO_FIFO_WR) = cmd;
            __dsb();
            __sev();
            break;
        }
        ++i;
    }
    if (i >= MULTICORE_TIMEOUT) { return false; }
    /* Wait for echo */
    i = 0u;
    while (i < MULTICORE_TIMEOUT) {
        __wfe();
        if ((REG_RO(SIO_BASE + SIO_FIFO_ST) & SIO_FIFO_ST_VLD) != 0u) {
            response = REG_RO(SIO_BASE + SIO_FIFO_RD);
            if (response == cmd) {
                (*seq)++;
            } else {
                *seq = 0u;
            }
            return true;
        }
        ++i;
    }
    return false;
}

/*
 * multicore_launch_core1 – RP2350 ROM 6-step FIFO boot protocol.
 *
 * The ROM on Core1 expects this exact sequence over the SIO FIFO:
 *   [0] 0      – drain / abort (Core1 flushes and echoes 0)
 *   [1] 0      – drain again  (Core1 echoes 0)
 *   [2] 1      – start vector handshake
 *   [3] VTOR   – Core1's vector-table base (same as Core0's)
 *   [4] SP     – initial stack pointer for Core1
 *   [5] entry  – Core1 entry address
 * Core1 echoes each word back; on mismatch the sequence restarts.
 */
bool multicore_launch_core1(core_entry_t entry, uint32_t *stack_top) {
    uint32_t seq;
    uint32_t cmd_seq[MULTICORE_CMD_COUNT];
    ASSERT(entry     != NULL);
    ASSERT(stack_top != NULL);

    cmd_seq[0] = 0u;
    cmd_seq[1] = 0u;
    cmd_seq[2] = 1u;
    cmd_seq[3] = REG_RO(SCB_VTOR);
    cmd_seq[4] = (uint32_t)(uintptr_t)stack_top;
    cmd_seq[5] = (uint32_t)(uintptr_t)entry;

    seq = 0u;
    while (seq < MULTICORE_CMD_COUNT) {
        uint32_t cmd = cmd_seq[seq];
        if (cmd == 0u) {
            fifo_drain_and_wake();
        }
        if (!fifo_cmd_exchange(cmd, &seq)) {
            return false;
        }
    }
    return true;
}
