// Second stage boot code for W25Q080 flash
// Copyright (c) 2019-2024 Raspberry Pi (Trading) Ltd.
// SPDX-License-Identifier: BSD-3-Clause
//
// Configures W25Q080 to run in Quad I/O continuous read XIP mode

.syntax unified
.cpu cortex-m33
.thumb

// Include register definitions
#include "boot_stage2_regs.h"

// Config section
#define PICO_FLASH_SPI_CLKDIV   2
#define PICO_FLASH_SPI_RXDELAY  2
#define CMD_READ                0xeb
#define MODE_CONTINUOUS_READ    0xa0
#define WAIT_CYCLES             4
#define CMD_WRITE_ENABLE        0x06
#define CMD_READ_STATUS         0x05
#define CMD_READ_STATUS2        0x35
#define CMD_WRITE_STATUS        0x01
#define SREG_DATA               0x02

// Register initialization values
#define INIT_PAD_SCLK   (2 << PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_LSB | PADS_QSPI_GPIO_QSPI_SCLK_SLEWFAST_BITS)
#define INIT_DIRECT_CSR (30 << QMI_DIRECT_CSR_CLKDIV_LSB | QMI_DIRECT_CSR_EN_BITS | QMI_DIRECT_CSR_AUTO_CS0N_BITS)
#define INIT_M0_TIMING  (1 << QMI_M0_TIMING_COOLDOWN_LSB | PICO_FLASH_SPI_RXDELAY << QMI_M0_TIMING_RXDELAY_LSB | PICO_FLASH_SPI_CLKDIV << QMI_M0_TIMING_CLKDIV_LSB)
#define INIT_M0_RCMD    (CMD_READ << QMI_M0_RCMD_PREFIX_LSB | MODE_CONTINUOUS_READ << QMI_M0_RCMD_SUFFIX_LSB)
#define INIT_M0_RFMT    (QMI_M0_RFMT_PREFIX_WIDTH_VALUE_S << QMI_M0_RFMT_PREFIX_WIDTH_LSB | QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_RFMT_ADDR_WIDTH_LSB | QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB | QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_RFMT_DUMMY_WIDTH_LSB | QMI_M0_RFMT_DATA_WIDTH_VALUE_Q << QMI_M0_RFMT_DATA_WIDTH_LSB | QMI_M0_RFMT_PREFIX_LEN_VALUE_8 << QMI_M0_RFMT_PREFIX_LEN_LSB | QMI_M0_RFMT_SUFFIX_LEN_VALUE_8 << QMI_M0_RFMT_SUFFIX_LEN_LSB | WAIT_CYCLES << QMI_M0_RFMT_DUMMY_LEN_LSB)

.section .text

.global _stage2_boot
.type _stage2_boot,%function
.thumb_func
_stage2_boot:
    push {lr}

_pad_config:
    ldr r3, =PADS_QSPI_BASE
    movs r0, INIT_PAD_SCLK
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SCLK_OFFSET]
    
    // Disable input Schmitt on SDx pins
    adds r3, #REG_ALIAS_CLR_BITS
    movs r0, #PADS_QSPI_GPIO_QSPI_SD0_SCHMITT_BITS
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SD0_OFFSET]
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SD1_OFFSET]
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SD2_OFFSET]
    str r0, [r3, #PADS_QSPI_GPIO_QSPI_SD3_OFFSET]
    
    adds r3, (XIP_QMI_BASE - (PADS_QSPI_BASE + REG_ALIAS_CLR_BITS))

program_sregs:
    ldr r1, =INIT_DIRECT_CSR
    str r1, [r3, #QMI_DIRECT_CSR_OFFSET]
    
    // Wait for cooldown
1:
    ldr r0, [r3, #QMI_DIRECT_CSR_OFFSET]
    tst r0, #QMI_DIRECT_CSR_BUSY_BITS
    bne 1b
    
    // Check status register 2
    movs r0, #CMD_READ_STATUS2
    bl read_flash_sreg
    cmp r0, #SREG_DATA
    beq _skip_sreg_programming
    
    // Write enable
    movs r0, #CMD_WRITE_ENABLE
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    bl wait_qmi_ready
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]
    
    // Write status
    movs r0, #CMD_WRITE_STATUS
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    movs r0, #0
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    movs r0, #SREG_DATA
    str r0, [r3, #QMI_DIRECT_TX_OFFSET]
    bl wait_qmi_ready
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]
    ldr r0, [r3, #QMI_DIRECT_RX_OFFSET]
    
    // Wait for write complete
1:
    movs r0, #CMD_READ_STATUS
    bl read_flash_sreg
    lsrs r0, #1
    bcs 1b

_skip_sreg_programming:
    // Disable direct mode
    bics r1, #QMI_DIRECT_CSR_EN_BITS
    str r1, [r3, #QMI_DIRECT_CSR_OFFSET]

_qmi_config:
    ldr r0, =INIT_M0_TIMING
    str r0, [r3, #QMI_M0_TIMING_OFFSET]
    ldr r0, =INIT_M0_RCMD
    str r0, [r3, #QMI_M0_RCMD_OFFSET]
    ldr r0, =INIT_M0_RFMT
    str r0, [r3, #QMI_M0_RFMT_OFFSET]
    
    // Dummy transfer
    mov r1, #XIP_NOCACHE_NOALLOC_BASE
    ldrb r1, [r1]
    
    // Set prefix length to 0
    bic r0, #QMI_M0_RFMT_PREFIX_LEN_BITS
    str r0, [r3, #QMI_M0_RFMT_OFFSET]

    // Exit via saved lr
    pop {pc}

#include "wait_qmi_ready.S"
#include "read_flash_sreg.S"

.global literals
literals:
.ltorg
