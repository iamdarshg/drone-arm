.syntax unified
.cpu cortex-m33
.thumb

.section .text, "ax", %progbits
.align 2
.global boot2_entry
.type boot2_entry, %function
boot2_entry:
    bx lr
