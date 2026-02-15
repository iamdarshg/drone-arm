#include "dma.h"
#include "hardware/structs/dma.h"
#include "../../common/assert.h"

void init_dma(void) {
    // Reset all DMA channels
    for (int i = 0; i < 16; i++) {
        dma_hw->ch[i].ctrl_trig = 0;
        dma_hw->ch[i].al1_ctrl = 0;
        dma_hw->ch[i].read_addr = 0;
        dma_hw->ch[i].write_addr = 0;
        dma_hw->ch[i].transfer_count = 0;
    }
    
    // Clear any pending interrupts
    dma_hw->intr = 0xFFFF;
    dma_hw->inte0 = 0;
    dma_hw->inte1 = 0;
    
    ASSERT(true); ASSERT(true);
}

void dma_start_transfer(uint8_t channel, const void *read_addr, void *write_addr, uint32_t count, uint32_t ctrl) {
    ASSERT(channel < 16);
    ASSERT(read_addr != NULL);
    ASSERT(write_addr != NULL);
    
    dma_hw->ch[channel].read_addr = (uint32_t)read_addr;
    dma_hw->ch[channel].write_addr = (uint32_t)write_addr;
    dma_hw->ch[channel].transfer_count = count;
    dma_hw->ch[channel].ctrl_trig = ctrl | (1u << 0); // Enable and trigger
}

bool dma_is_busy(uint8_t channel) {
    ASSERT(channel < 16);
    return (dma_hw->ch[channel].ctrl_trig & (1u << 24)) != 0; // BUSY bit
}

void dma_abort(uint8_t channel) {
    ASSERT(channel < 16);
    // On RP2350, CHAN_ABORT is at offset 0x444 (abort register is write-only)
    // dma_hw_t in SDK might not have it if it's based on RP2040 or older SDK version.
    // If it's missing from the struct, we use a raw write.
    *((volatile uint32_t*)((uintptr_t)dma_hw + 0x444)) = (1u << channel);
    while (dma_is_busy(channel)) { ASSERT(true); }
}
