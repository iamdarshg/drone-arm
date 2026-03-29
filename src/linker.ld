MEMORY
{
    ROM          (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
    FLASH        (rx)  : ORIGIN = 0x10000000, LENGTH = 2048K
    XIP_SRAM     (rwx) : ORIGIN = 0x13FFC000, LENGTH = 16K
    SRAM         (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
    SCRATCH_X    (rwx) : ORIGIN = 0x20080000, LENGTH = 4K
    SCRATCH_Y    (rwx) : ORIGIN = 0x20081000, LENGTH = 4K
}

ENTRY(reset_handler)

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    .vectors :
    {
        KEEP(*(.vectors))
    } > FLASH

    .boot2 :
    {
        KEEP(*(.boot2))
    } > FLASH

    .text :
    {
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
    } > FLASH

    .rodata :
    {
        *(.rodata*)
    } > FLASH

    _sidata = LOADADDR(.data);
    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM AT > FLASH

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    _siramfunc_load = LOADADDR(.ramfunc);
    .ramfunc :
    {
        _siramfunc = .;
        *(.ramfunc*)
        _eiramfunc = .;
    } > SRAM AT > FLASH

    .scratch_x (NOLOAD) :
    {
        *(.scratch_x*)
    } > SCRATCH_X

    .scratch_y (NOLOAD) :
    {
        *(.scratch_y*)
    } > SCRATCH_Y

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
    }
}
