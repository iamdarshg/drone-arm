#ifndef SPI_CONFIG_H
#define SPI_CONFIG_H

#include <stdint.h>

// SPI Interface IDs (runtime selectable)
#define SPI_ID_0        0
#define SPI_ID_1        1
#define SPI_NUM_INTERFACES  2

// SPI Base clock - CLK_PERI (typically 48MHz or 150MHz)
// Maximum SPI speed calculation:
// Bit rate = CLK_PERI / (CPSDVSR * (1 + SCR))
// For max speed: CPSDVSR = 2 (minimum even), SCR = 0
// At 150MHz CLK_PERI: 150/2 = 75MHz max
// At 48MHz CLK_PERI: 48/2 = 24MHz max
#define SPI_CLK_PERI_FREQ   150000000   // 150 MHz - adjust to actual clock

// Maximum SPI clock settings
#define SPI_CPSDVSR_MIN     2           // Minimum prescale divisor (must be even)
#define SPI_SCR_MIN         0           // Minimum serial clock rate

// Default SPI configuration
#define SPI_DEFAULT_BAUD    10000000    // 10 MHz default
#define SPI_MAX_BAUD        (SPI_CLK_PERI_FREQ / SPI_CPSDVSR_MIN)  // 75 MHz @ 150MHz

// SPI Frame format
#define SPI_FRF_MOTOROLA    0x0         // Motorola SPI (standard)
#define SPI_FRF_TI          0x1         // TI synchronous serial
#define SPI_FRF_MICROWIRE   0x2         // National Microwire

// SPI Data size
#define SPI_DSS_8BIT        0x7         // 8-bit data
#define SPI_DSS_16BIT       0xF         // 16-bit data

// SPI Mode (CPOL, CPHA)
#define SPI_MODE_0          0x0         // CPOL=0, CPHA=0
#define SPI_MODE_1          0x1         // CPOL=0, CPHA=1
#define SPI_MODE_2          0x2         // CPOL=1, CPHA=0
#define SPI_MODE_3          0x3         // CPOL=1, CPHA=1

// Status register bits
#define SPI_SR_BSY          (1U << 4)   // Busy
#define SPI_SR_RFF          (1U << 3)   // Receive FIFO full
#define SPI_SR_RNE          (1U << 2)   // Receive FIFO not empty
#define SPI_SR_TNF          (1U << 1)   // Transmit FIFO not full
#define SPI_SR_TFE          (1U << 0)   // Transmit FIFO empty

#endif // SPI_CONFIG_H
