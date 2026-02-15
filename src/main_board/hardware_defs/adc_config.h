#ifndef ADC_CONFIG_H
#define ADC_CONFIG_H

#include <stdint.h>

// ADC Channel Mapping for RP2350
// ADC1 = GPIO41, ADC3 = GPIO43, ADC5 = GPIO45, ADC7 = GPIO47
#define ADC_CHANNEL_1_PIN   41
#define ADC_CHANNEL_3_PIN   43
#define ADC_CHANNEL_5_PIN   45
#define ADC_CHANNEL_7_PIN   47

// Active channels mask for round-robin
// Bits 1, 3, 5, 7 set = 0x000000AA
#define ADC_RROBIN_MASK     0x000000AA
#define ADC_NUM_CHANNELS    4

// DMA Configuration
#define ADC_DMA_CHANNEL     0           // DMA channel 0 for ADC
#define ADC_DMA_IRQ         0           // IRQ line for DMA completion

// Sampling Configuration
// Target: 10kSPS per channel = 40kSPS total
// ADC clock = 48MHz, conversion takes 96 cycles = 500kSPS max
// With 4 channels in round-robin: 500k/4 = 125kSPS per channel
// We use clock divider to achieve exactly 40kSPS total (10k per channel)
// Divider = 48MHz / (40k * 96) = 12.5
// DIV.INT = 12, DIV.FRAC = 128 (0x80) for 12.5
#define ADC_DIV_INT         12
#define ADC_DIV_FRAC        128

// FIFO Configuration
#define ADC_FIFO_SIZE       4           // ADC FIFO depth
#define ADC_DMA_BUFFER_SAMPLES  256     // Samples per buffer (must be multiple of 4 for round-robin)
#define ADC_DMA_BUFFER_BYTES    (ADC_DMA_BUFFER_SAMPLES * 2)  // 16-bit samples

// Temperature sensor (on ADC4)
#define ADC_TEMP_CHANNEL    4
#define ADC_TEMP_EN_BIT     (1U << 1)   // TS_EN bit in CS register

#endif // ADC_CONFIG_H
