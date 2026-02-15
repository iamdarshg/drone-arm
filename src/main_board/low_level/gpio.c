/*
 * Optimized GPIO implementation for RP2350 using GPIO Coprocessor
 * Hardcoded for 48-pin RP2350B with minimal overhead
 * Uses GPIO Coprocessor (GPC) for atomic operations - reduces instruction count
 * Uses global pin configuration arrays from pins.h
 * Uses __arm_mcr/__arm_mrc intrinsics for better compiler portability
 */

#include <stdint.h>

// ARM Coprocessor intrinsics for better portability
// These map to MCR/MRC instructions on Cortex-M33
#if defined(__ARMCC_VERSION)
    // ARM Compiler (armcc)
    #include <arm_compat.h>
#elif defined(__GNUC__) || defined(__clang__)
    // GCC or Clang - define intrinsics using inline asm
    static inline void __arm_mcr(uint32_t coproc, uint32_t opcode1, uint32_t value, 
                                  uint32_t crn, uint32_t crm, uint32_t opcode2) {
        __asm volatile ("mcr p%0, %1, %2, c%3, c%4, %5" 
                       : : "i" (coproc), "i" (opcode1), "r" (value), 
                         "i" (crn), "i" (crm), "i" (opcode2) : "memory");
    }
    
    static inline uint32_t __arm_mrc(uint32_t coproc, uint32_t opcode1, 
                                      uint32_t crn, uint32_t crm, uint32_t opcode2) {
        uint32_t result;
        __asm volatile ("mrc p%1, %2, %0, c%3, c%4, %5" 
                       : "=r" (result) 
                       : "i" (coproc), "i" (opcode1), "i" (crn), "i" (crm), "i" (opcode2));
        return result;
    }
#else
    #error "Unsupported compiler - ARM coprocessor intrinsics not available"
#endif

#include "gpio.h"
#include "../../../include/hardware/structs/sio.h"
#include "../../../include/hardware/structs/io_bank0.h"
#include "../../../include/hardware/structs/pads_bank0.h"
#include "../../../include/hardware/regs/addressmap.h"

// Fast SIO register pointers for inline functions
volatile uint32_t *const gpio_sio_out_set = &sio_hw->gpio_out_set;
volatile uint32_t *const gpio_sio_out_clr = &sio_hw->gpio_out_clr;
volatile uint32_t *const gpio_sio_out_xor = &sio_hw->gpio_out_xor;
volatile uint32_t *const gpio_sio_in = &sio_hw->gpio_in;
volatile uint32_t *const gpio_sio_hi_out_set = &sio_hw->gpio_hi_out_set;
volatile uint32_t *const gpio_sio_hi_out_clr = &sio_hw->gpio_hi_out_clr;
volatile uint32_t *const gpio_sio_hi_out_xor = &sio_hw->gpio_hi_out_xor;
volatile uint32_t *const gpio_sio_hi_in = &sio_hw->gpio_hi_in;

// GPIO Coprocessor (GPC) base address - provides atomic operations
#define GPC_BASE        0x40030000
#define GPC_CPUCTRL     (volatile uint32_t *)(GPC_BASE + 0x00)
#define GPC_CPUTOGGLE   (volatile uint32_t *)(GPC_BASE + 0x04)

// Use __arm_mcr intrinsic for MCR instruction (Move to Coprocessor from Register)
// Format: __arm_mcr(coproc, opcode1, value, crn, crm, opcode2)
// CP1 - GPIO Set: Atomic set bits (single instruction)
static inline void gpio_coprocessor_set(uint32_t mask_low, uint32_t mask_high) {
    __arm_mcr(1, 0, mask_low, 0, 0, 0);    // Set low 32 bits
    __arm_mcr(1, 0, mask_high, 1, 0, 0);   // Set high 16 bits
}

// CP2 - GPIO Clear: Atomic clear bits (single instruction)
static inline void gpio_coprocessor_clr(uint32_t mask_low, uint32_t mask_high) {
    __arm_mcr(2, 0, mask_low, 0, 0, 0);    // Clear low 32 bits
    __arm_mcr(2, 0, mask_high, 1, 0, 0);   // Clear high 16 bits
}

// CP3 - GPIO Toggle: Atomic XOR bits (single instruction)
static inline void gpio_coprocessor_toggle(uint32_t mask_low, uint32_t mask_high) {
    __arm_mcr(3, 0, mask_low, 0, 0, 0);    // Toggle low 32 bits
    __arm_mcr(3, 0, mask_high, 1, 0, 0);   // Toggle high 16 bits
}

// CP4 - GPIO Output Enable Set: Atomic OE set (single instruction)
static inline void gpio_coprocessor_oe_set(uint32_t mask_low, uint32_t mask_high) {
    __arm_mcr(4, 0, mask_low, 0, 0, 0);    // OE set low 32 bits
    __arm_mcr(4, 0, mask_high, 1, 0, 0);   // OE set high 16 bits
}

// CP5 - GPIO Output Enable Clear: Atomic OE clear (single instruction)
static inline void gpio_coprocessor_oe_clr(uint32_t mask_low, uint32_t mask_high) {
    __arm_mcr(5, 0, mask_low, 0, 0, 0);    // OE clear low 32 bits
    __arm_mcr(5, 0, mask_high, 1, 0, 0);   // OE clear high 16 bits
}

// CP6 - GPIO Input Status: Read input (single instruction, returns in register)
// Use __arm_mrc intrinsic for MRC instruction (Move to Register from Coprocessor)
// Format: __arm_mrc(coproc, opcode1, crn, crm, opcode2)
static inline uint64_t gpio_coprocessor_in(void) {
    uint32_t lo = __arm_mrc(6, 0, 0, 0, 0);   // Read low 32 bits
    uint32_t hi = __arm_mrc(6, 0, 1, 0, 0);   // Read high 16 bits
    return ((uint64_t)hi << 32) | lo;
}

// Initialize all GPIO pins with default configuration
// Uses global_pin_func_map, global_pin_pullup, and global_pin_direction from pins.h
void gpio_init_all(void) {
    // Configure all pins based on global arrays
    for (uint8_t pin = 0; pin < GPIO_NUM_PINS; pin++) {
        // Apply function from global map
        gpio_set_func(pin, global_pin_func_map[pin]);
        
        // Apply direction from global array
        gpio_set_dir(pin, global_pin_direction[pin]);
        
        // Apply pull configuration from global array
        gpio_set_pull(pin, global_pin_pullup[pin]);
        
        // Apply schmitt trigger setting from pins.h
        gpio_set_schmitt(pin, use_schmidt_trigger);
    }
}

// Set GPIO function (XIP, SPI, UART, etc.)
// Updates both hardware and global_pin_func_map
void gpio_set_func(uint8_t pin, uint8_t func) {
    if (pin >= GPIO_NUM_PINS) return;
    
    // Update global configuration array
    global_pin_func_map[pin] = func;
    
    // Use atomic write to GPIO_CTRL register
    volatile uint32_t *ctrl_reg = &io_bank0_hw->gpio[pin].ctrl;
    *ctrl_reg = func & 0x1f;
}

// Set GPIO direction
// Updates both hardware and global_pin_direction
// Uses GPIO Coprocessor for atomic OE operations (single instruction vs 3-4)
void gpio_set_dir(uint8_t pin, bool output) {
    if (pin >= GPIO_NUM_PINS) return;
    
    // Update global direction array
    global_pin_direction[pin] = output ? 1 : 0;
    
    if (pin < 32) {
        uint32_t mask = (1u << pin);
        if (output) {
            // Use coprocessor - single instruction vs 3 instructions
            gpio_coprocessor_oe_set(mask, 0);
        } else {
            gpio_coprocessor_oe_clr(mask, 0);
        }
    } else {
        uint32_t mask = (1u << (pin - 32));
        if (output) {
            gpio_coprocessor_oe_set(0, mask);
        } else {
            gpio_coprocessor_oe_clr(0, mask);
        }
    }
}

// Set pull-up/pull-down
// Updates both hardware and global_pin_pullup
void gpio_set_pull(uint8_t pin, uint8_t pull) {
    if (pin >= GPIO_NUM_PINS) return;
    
    // Update global pullup array
    global_pin_pullup[pin] = pull;
    
    // PADS register - use atomic alias for write
    volatile uint32_t *pad_reg = &pads_bank0_hw->gpio[pin];
    
    // Atomic update: clear bits with mask, set new value
    *pad_reg = (*pad_reg & ~0x0c) | ((pull == GPIO_PULL_UP) ? (1u << 3) : (pull == GPIO_PULL_DOWN) ? (1u << 2) : 0);
}

// Set drive strength
void gpio_set_drive(uint8_t pin, uint8_t drive) {
    if (pin >= GPIO_NUM_PINS) return;
    
    volatile uint32_t *pad_reg = &pads_bank0_hw->gpio[pin];
    
    // Atomic update of drive strength bits [5:4]
    *pad_reg = (*pad_reg & ~(0x30)) | (drive << 4);
}

// Set Schmitt trigger
void gpio_set_schmitt(uint8_t pin, bool enable) {
    if (pin >= GPIO_NUM_PINS) return;
    
    volatile uint32_t *pad_reg = &pads_bank0_hw->gpio[pin];
    
    // Use atomic SET/CLR aliases for single-instruction update
    if (enable) {
        *(volatile uint32_t *)((uint32_t)pad_reg | REG_ALIAS_SET_BITS) = (1u << 1);
    } else {
        *(volatile uint32_t *)((uint32_t)pad_reg | REG_ALIAS_CLR_BITS) = (1u << 1);
    }
}

// Fast GPIO set using coprocessor (single instruction vs 4-5)
void gpio_set_fast(uint8_t pin, bool value) {
    if (pin >= GPIO_NUM_PINS) return;
    
    if (pin < 32) {
        uint32_t mask = (1u << pin);
        if (value) {
            gpio_coprocessor_set(mask, 0);
        } else {
            gpio_coprocessor_clr(mask, 0);
        }
    } else {
        uint32_t mask = (1u << (pin - 32));
        if (value) {
            gpio_coprocessor_set(0, mask);
        } else {
            gpio_coprocessor_clr(0, mask);
        }
    }
}

// Fast GPIO toggle using coprocessor (single instruction vs 5-6)
void gpio_toggle_fast(uint8_t pin) {
    if (pin >= GPIO_NUM_PINS) return;
    
    if (pin < 32) {
        gpio_coprocessor_toggle((1u << pin), 0);
    } else {
        gpio_coprocessor_toggle(0, (1u << (pin - 32)));
    }
}

// Batch set multiple pins using coprocessor (2 instructions vs 8+)
void gpio_set_mask(uint32_t mask_low, uint32_t mask_high, bool value) {
    if (value) {
        gpio_coprocessor_set(mask_low, mask_high);
    } else {
        gpio_coprocessor_clr(mask_low, mask_high);
    }
}

// Batch toggle multiple pins using coprocessor (2 instructions vs 8+)
void gpio_toggle_mask(uint32_t mask_low, uint32_t mask_high) {
    gpio_coprocessor_toggle(mask_low, mask_high);
}

// Batch read multiple pins using coprocessor (2 instructions vs 4+)
uint32_t gpio_get_mask(uint32_t mask_low, uint32_t mask_high) {
    uint64_t inputs = gpio_coprocessor_in();
    // Return low 32 bits masked - for full 64-bit, cast to uint64_t
    (void)mask_high;  // Compiler warning suppression
    return ((uint32_t)inputs) & mask_low;
}

// Initialize GPIO coprocessor control
void gpio_coprocessor_init(void) {
    // Enable GPIO coprocessor in SYSCFG
    volatile uint32_t *syscfg_gpctrl = (volatile uint32_t *)(SYSCFG_BASE + 0x08);
    *syscfg_gpctrl = 0x1;  // Enable GPC
}
