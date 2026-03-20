#ifndef PLACEMENT_H
#define PLACEMENT_H

#include <stdint.h>
#include <stddef.h>

/* Attribute to place a function or data into the ram-executed section.
 * These symbols are defined in the linker script and result in the
 * contents being loaded from flash and copied to RAM at startup. */
#define ATTR_RAMFUNC __attribute__((section(".ramfunc")))

/* Attribute to place large read-only or flash-resident data into flash storage
 * area (kept in flash, not copied into RAM). Useful for logs/snapshots/tables. */
#define ATTR_FLASH_STORAGE __attribute__((section(".flash_storage")))

/* Linker-provided symbols for flash storage region */
extern uint8_t __sflash_storage[];
extern uint8_t __eflash_storage[];

static inline void *flash_storage_start(void) {
    return (void *)__sflash_storage;
}

static inline size_t flash_storage_size(void) {
    return (size_t)(&__eflash_storage - &__sflash_storage);
}

#endif /* PLACEMENT_H */
