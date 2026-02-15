/*
 * SPI hardware structs - Motorola mode only (hardcoded)
 */
#ifndef _HARDWARE_STRUCTS_SPI_H
#define _HARDWARE_STRUCTS_SPI_H

#include "../address_mapped.h"
#include "../regs/addressmap.h"

// SPI register offsets
#define SPI_SSPCR0_OFFSET       0x00
#define SPI_SSPCR1_OFFSET       0x04
#define SPI_SSPDR_OFFSET        0x08
#define SPI_SSPSR_OFFSET        0x0c
#define SPI_SSPCPSR_OFFSET      0x10
#define SPI_SSPIMSC_OFFSET      0x14
#define SPI_SSPRIS_OFFSET       0x18
#define SPI_SSPMIS_OFFSET       0x1c
#define SPI_SSPICR_OFFSET       0x20
#define SPI_SSPDMACR_OFFSET     0x24

// SPI hardware register block
typedef struct {
    io_rw_32 cr0;       // Control register 0
    io_rw_32 cr1;       // Control register 1
    io_rw_32 dr;        // Data register
    io_ro_32 sr;        // Status register
    io_rw_32 cpsr;      // Clock prescale
    io_rw_32 imsc;      // Interrupt mask
    io_ro_32 ris;       // Raw interrupt status
    io_ro_32 mis;       // Masked interrupt status
    io_rw_32 icr;       // Interrupt clear
    io_rw_32 dmacr;     // DMA control
} spi_hw_t;

#define spi0_hw ((spi_hw_t *const)SPI0_BASE)
#define spi1_hw ((spi_hw_t *const)SPI1_BASE)

// CR0 bits
#define SPI_CR0_DSS_LSB         0
#define SPI_CR0_DSS_BITS        0x0f
#define SPI_CR0_FRF_LSB         4
#define SPI_CR0_FRF_BITS        0x30
#define SPI_CR0_SPO_BIT         (1u << 6)
#define SPI_CR0_SPH_BIT         (1u << 7)
#define SPI_CR0_SCR_LSB         8
#define SPI_CR0_SCR_BITS        0xff00

// CR1 bits
#define SPI_CR1_LBM_BIT         (1u << 0)
#define SPI_CR1_SSE_BIT         (1u << 1)
#define SPI_CR1_MS_BIT          (1u << 2)
#define SPI_CR1_SOD_BIT         (1u << 3)

// Status bits
#define SPI_SR_TFE_BIT          (1u << 0)  // TX FIFO empty
#define SPI_SR_TNF_BIT          (1u << 1)  // TX FIFO not full
#define SPI_SR_RNE_BIT          (1u << 2)  // RX FIFO not empty
#define SPI_SR_RFF_BIT          (1u << 3)  // RX FIFO full
#define SPI_SR_BSY_BIT          (1u << 4)  // Busy

// Hardcoded for Motorola SPI mode (FRF = 0)
#define SPI_MODE_MOTOROLA       0

#endif // _HARDWARE_STRUCTS_SPI_H
