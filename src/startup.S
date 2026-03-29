.syntax unified
.cpu cortex-m33
.thumb

.global reset_handler
.global default_handler
.global __vectors

.extern _estack
.extern _sidata
.extern _sdata
.extern _edata
.extern _sbss
.extern _ebss
.extern _siramfunc_load
.extern _siramfunc
.extern _eiramfunc
.extern main

.section .vectors, "a", %progbits
.align 2
__vectors:
    .word _estack
    .word reset_handler
    .word default_handler
    .word default_handler
    .word default_handler
    .word default_handler
    .word default_handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word default_handler
    .word default_handler
    .word 0
    .word default_handler
    .word default_handler

.section .text.reset_handler, "ax", %progbits
.thumb_func
reset_handler:
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
1:
    cmp r1, r2
    bcs 2f
    ldr r3, [r0]
    str r3, [r1]
    adds r0, r0, #4
    adds r1, r1, #4
    b 1b

2:
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
3:
    cmp r0, r1
    bcs 4f
    str r2, [r0]
    adds r0, r0, #4
    b 3b

4:
    ldr r0, =_siramfunc_load
    ldr r1, =_siramfunc
    ldr r2, =_eiramfunc
5:
    cmp r1, r2
    bcs 6f
    ldr r3, [r0]
    str r3, [r1]
    adds r0, r0, #4
    adds r1, r1, #4
    b 5b

6:
    bl main
7:
    b 7b

.thumb_func
default_handler:
8:
    b 8b
