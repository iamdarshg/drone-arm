/*
 * Linker script for RP2350 Main Board
 * Reserves last 4KB of flash for self-test area
 */

MEMORY {
    /* Flash memory: 2MB total, reserve last 4KB for testing */
    FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 0x1FF000  /* 2MB - 4KB = 2044KB */
    
    /* Flash test area: last 4KB of flash */
    FLASH_TEST(rwx) : ORIGIN = 0x101FF000, LENGTH = 0x1000  /* 4KB at end of flash */
    
    /* SRAM */
    RAM(rwx) : ORIGIN = 0x20000000, LENGTH = 512K
    
    /* Boot RAM */
    BOOTRAM(rwx) : ORIGIN = 0x20080000, LENGTH = 8K
    
    /* XIP cached memory */
    XIP(rwx) : ORIGIN = 0x13ffc000, LENGTH = 16K
}

SECTIONS {
    /* Text section in flash */
    .text : {
        _stext = .;
        KEEP(*(.entry))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH
    
    /* Initialized data */
    .data : {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH
    
    /* BSS */
    .bss : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM
    
    /* Stack at end of RAM */
    __stack_top = ORIGIN(RAM) + LENGTH(RAM);
}

/* Export flash test region address */
PROVIDE(flash_test_region = ORIGIN(FLASH_TEST));
