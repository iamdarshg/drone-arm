#include "hardware/regs/addressmap.h"
#include "mpu.h"
#include "hardware/structs/mpu.h"
#include "assert.h"
#include <stddef.h>

void init_mpu(void) {
    // Basic MPU setup for RP2350 (Cortex-M33)
    // 1. Disable MPU
    mpu_hw->ctrl &= ~1;
    
    // 2. Configure regions (Example: Flash read-only, RAM no-exec, etc.)
    // Region 0: Flash (0x10000000)
    mpu_hw->rnr = 0;
    mpu_hw->rbar = 0x10000000 | (3 << 1) | 1; // Read-only, executable
    mpu_hw->rlar = (0x10000000 + 0x01000000 - 1) | 1; // 16MB limit
    
    // 3. Enable MPU with default memory map as background
    mpu_hw->ctrl |= (1 << 2) | 1;
    
    ASSERT(mpu_hw->ctrl & 1);
    ASSERT(true);
}
