#ifndef ADC_H
#define ADC_H

 #include <stdint.h>
 #include <stdbool.h>
 #include "../../../include/hardware/structs/adc.h"
 #include "../../../include/hardware/structs/dma.h"
 #include "../../../include/hardware/regs/addressmap.h"
 #include "../hardware_defs/adc_config.h"

/** @brief Standardized ADC initialization wrapper. */
void init_adc(void);

void adc_init(void);

// ============================================================================
// ADC Register Bit Definitions
// ============================================================================

// CS Register bits
#ifndef ADC_CS_EN_BIT
#define ADC_CS_EN_BIT           (1U << 0)
#endif
#ifndef ADC_CS_TS_EN_BIT
#define ADC_CS_TS_EN_BIT        (1U << 1)
#endif
#ifndef ADC_CS_START_ONCE_BIT
#define ADC_CS_START_ONCE_BIT   (1U << 2)
#endif
#ifndef ADC_CS_START_MANY_BIT
#define ADC_CS_START_MANY_BIT   (1U << 3)
#endif
#ifndef ADC_CS_READY_BIT
#define ADC_CS_READY_BIT        (1U << 8)
#endif
#ifndef ADC_CS_ERR_BIT
#define ADC_CS_ERR_BIT          (1U << 9)
#endif
#ifndef ADC_CS_ERR_STICKY_BIT
#define ADC_CS_ERR_STICKY_BIT   (1U << 10)
#endif
#ifndef ADC_CS_AINSEL_SHIFT
#define ADC_CS_AINSEL_SHIFT     12
#endif
#ifndef ADC_CS_AINSEL_MASK
#define ADC_CS_AINSEL_MASK      (0xFU << ADC_CS_AINSEL_SHIFT)
#endif
#ifndef ADC_CS_RROBIN_SHIFT
#define ADC_CS_RROBIN_SHIFT     16
#endif
#ifndef ADC_CS_RROBIN_MASK
#define ADC_CS_RROBIN_MASK      (0x1FFU << ADC_CS_RROBIN_SHIFT)
#endif

// FCS Register bits
#ifndef ADC_FCS_EN_BIT
#define ADC_FCS_EN_BIT          (1U << 0)
#endif
#ifndef ADC_FCS_SHIFT_BIT
#define ADC_FCS_SHIFT_BIT       (1U << 1)
#endif
#ifndef ADC_FCS_ERR_BIT
#define ADC_FCS_ERR_BIT         (1U << 2)
#endif
#ifndef ADC_FCS_DREQ_EN_BIT
#define ADC_FCS_DREQ_EN_BIT     (1U << 3)
#endif
#ifndef ADC_FCS_EMPTY_BIT
#define ADC_FCS_EMPTY_BIT       (1U << 8)
#endif
#ifndef ADC_FCS_FULL_BIT
#define ADC_FCS_FULL_BIT        (1U << 9)
#endif
#ifndef ADC_FCS_UNDER_BIT
#define ADC_FCS_UNDER_BIT       (1U << 10)
#endif
#ifndef ADC_FCS_OVER_BIT
#define ADC_FCS_OVER_BIT        (1U << 11)
#endif
#ifndef ADC_FCS_LEVEL_SHIFT
#define ADC_FCS_LEVEL_SHIFT     16
#endif
#ifndef ADC_FCS_LEVEL_MASK
#define ADC_FCS_LEVEL_MASK      (0xFU << ADC_FCS_LEVEL_SHIFT)
#endif
#ifndef ADC_FCS_THRESH_SHIFT
#define ADC_FCS_THRESH_SHIFT    24
#endif
#ifndef ADC_FCS_THRESH_MASK
#define ADC_FCS_THRESH_MASK     (0xFU << ADC_FCS_THRESH_SHIFT)
#endif

// FIFO Register bits
#ifndef ADC_FIFO_VAL_MASK
#define ADC_FIFO_VAL_MASK       0xFFFU
#endif
#ifndef ADC_FIFO_ERR_BIT
#define ADC_FIFO_ERR_BIT        (1U << 15)
#endif

// DIV Register shifts
#ifndef ADC_DIV_FRAC_SHIFT
#define ADC_DIV_FRAC_SHIFT      0
#endif
#ifndef ADC_DIV_FRAC_MASK
#define ADC_DIV_FRAC_MASK       0xFFU
#endif
#ifndef ADC_DIV_INT_SHIFT
#define ADC_DIV_INT_SHIFT       8
#endif
#ifndef ADC_DIV_INT_MASK
#define ADC_DIV_INT_MASK        (0xFFFFU << ADC_DIV_INT_SHIFT)
#endif

// ============================================================================
// DMA Configuration for ADC
// ============================================================================

// DMA Transfer request select for ADC
#define DMA_TREQ_ADC            0x24    // DREQ_ADC

// DMA Channel control bits
#define DMA_CTRL_EN_BIT         (1U << 0)
#define DMA_CTRL_HIGH_PRIORITY  (1U << 1)
#define DMA_CTRL_DATA_SIZE_BYTE (0U << 2)
#define DMA_CTRL_DATA_SIZE_HALF (1U << 2)
#define DMA_CTRL_DATA_SIZE_WORD (2U << 2)
#define DMA_CTRL_INCR_READ      (1U << 4)
#define DMA_CTRL_INCR_WRITE     (1U << 6)
#define DMA_CTRL_DREQ           (1U << 6)   // Use DREQ pacing

// ============================================================================
// Inline Functions - ADC Control
// ============================================================================

static inline void adc_enable(void) {
    adc_hw->cs |= ADC_CS_EN_BIT;
}

static inline void adc_disable(void) {
    adc_hw->cs &= ~ADC_CS_EN_BIT;
}

static inline void adc_temp_sensor_enable(void) {
    adc_hw->cs |= ADC_CS_TS_EN_BIT;
}

static inline void adc_temp_sensor_disable(void) {
    adc_hw->cs &= ~ADC_CS_TS_EN_BIT;
}

static inline void adc_start_once(void) {
    adc_hw->cs |= ADC_CS_START_ONCE_BIT;
}

static inline void adc_start_many(void) {
    adc_hw->cs |= ADC_CS_START_MANY_BIT;
}

static inline void adc_stop_many(void) {
    adc_hw->cs &= ~ADC_CS_START_MANY_BIT;
}

static inline bool adc_is_ready(void) {
    return (adc_hw->cs & ADC_CS_READY_BIT) != 0;
}

static inline bool adc_has_error(void) {
    return (adc_hw->cs & ADC_CS_ERR_BIT) != 0;
}

static inline void adc_clear_error_sticky(void) {
    adc_hw->cs |= ADC_CS_ERR_STICKY_BIT;
}

static inline void adc_select_channel(uint8_t channel) {
    uint32_t cs = adc_hw->cs;
    cs &= ~ADC_CS_AINSEL_MASK;
    cs |= (channel << ADC_CS_AINSEL_SHIFT) & ADC_CS_AINSEL_MASK;
    adc_hw->cs = cs;
}

static inline uint8_t adc_get_channel(void) {
    return (uint8_t)((adc_hw->cs & ADC_CS_AINSEL_MASK) >> ADC_CS_AINSEL_SHIFT);
}

static inline void adc_set_round_robin_mask(uint32_t mask) {
    uint32_t cs = adc_hw->cs;
    cs &= ~ADC_CS_RROBIN_MASK;
    cs |= (mask << ADC_CS_RROBIN_SHIFT) & ADC_CS_RROBIN_MASK;
    adc_hw->cs = cs;
}

static inline uint32_t adc_get_round_robin_mask(void) {
    return (adc_hw->cs & ADC_CS_RROBIN_MASK) >> ADC_CS_RROBIN_SHIFT;
}

// ============================================================================
// Inline Functions - Clock Divider
// ============================================================================

static inline void adc_set_clkdiv(uint16_t integer, uint8_t fraction) {
    uint32_t div = ((uint32_t)integer << ADC_DIV_INT_SHIFT) | 
                   ((uint32_t)fraction & ADC_DIV_FRAC_MASK);
    adc_hw->div = div;
}

static inline uint32_t adc_get_clkdiv(void) {
    return adc_hw->div;
}

// ============================================================================
// Inline Functions - FIFO Control
// ============================================================================

static inline void adc_fifo_enable(void) {
    adc_hw->fcs |= ADC_FCS_EN_BIT;
}

static inline void adc_fifo_disable(void) {
    adc_hw->fcs &= ~ADC_FCS_EN_BIT;
}

static inline void adc_fifo_set_shift(bool shift_8bit) {
    if (shift_8bit) {
        adc_hw->fcs |= ADC_FCS_SHIFT_BIT;
    } else {
        adc_hw->fcs &= ~ADC_FCS_SHIFT_BIT;
    }
}

static inline void adc_fifo_enable_dma_req(void) {
    adc_hw->fcs |= ADC_FCS_DREQ_EN_BIT;
}

static inline void adc_fifo_disable_dma_req(void) {
    adc_hw->fcs &= ~ADC_FCS_DREQ_EN_BIT;
}

static inline void adc_fifo_set_threshold(uint8_t threshold) {
    uint32_t fcs = adc_hw->fcs;
    fcs &= ~ADC_FCS_THRESH_MASK;
    fcs |= ((uint32_t)threshold << ADC_FCS_THRESH_SHIFT) & ADC_FCS_THRESH_MASK;
    adc_hw->fcs = fcs;
}

static inline uint8_t adc_fifo_get_level(void) {
    return (uint8_t)((adc_hw->fcs & ADC_FCS_LEVEL_MASK) >> ADC_FCS_LEVEL_SHIFT);
}

static inline bool adc_fifo_is_empty(void) {
    return (adc_hw->fcs & ADC_FCS_EMPTY_BIT) != 0;
}

static inline bool adc_fifo_is_full(void) {
    return (adc_hw->fcs & ADC_FCS_FULL_BIT) != 0;
}

static inline bool adc_fifo_has_overflow(void) {
    return (adc_hw->fcs & ADC_FCS_OVER_BIT) != 0;
}

static inline bool adc_fifo_has_underflow(void) {
    return (adc_hw->fcs & ADC_FCS_UNDER_BIT) != 0;
}

static inline void adc_fifo_clear_overflow(void) {
    adc_hw->fcs |= ADC_FCS_OVER_BIT;
}

static inline void adc_fifo_clear_underflow(void) {
    adc_hw->fcs |= ADC_FCS_UNDER_BIT;
}

// ============================================================================
// Inline Functions - FIFO Data Access
// ============================================================================

static inline uint16_t adc_fifo_read(void) {
    return (uint16_t)(adc_hw->fifo & ADC_FIFO_VAL_MASK);
}

static inline uint16_t adc_fifo_read_with_error(bool *error) {
    uint32_t fifo_val = adc_hw->fifo;
    if (error) {
        *error = (fifo_val & ADC_FIFO_ERR_BIT) != 0;
    }
    return (uint16_t)(fifo_val & ADC_FIFO_VAL_MASK);
}

static inline uint16_t adc_get_result(void) {
    return (uint16_t)(adc_hw->result & ADC_FIFO_VAL_MASK);
}

// ============================================================================
// Interrupt Control
// ============================================================================

static inline void adc_irq_enable(void) {
    adc_hw->inte = 1U;  // Enable FIFO interrupt
}

static inline void adc_irq_disable(void) {
    adc_hw->inte = 0U;
}

static inline void adc_irq_force(void) {
    adc_hw->intf = 1U;
}

static inline bool adc_irq_is_pending(void) {
    return (adc_hw->ints & 1U) != 0;
}

// ============================================================================
// Function Prototypes
// ============================================================================

void adc_init(void);
void adc_init_gpio(uint8_t gpio);
void adc_set_round_robin(bool enable);
uint16_t adc_read_blocking(uint8_t channel);
float adc_convert_to_voltage(uint16_t adc_value);
float adc_read_temperature(void);

// DMA-based continuous sampling
void adc_dma_init(uint16_t *buffer, uint32_t num_samples);
void adc_dma_start(void);
void adc_dma_stop(void);
bool adc_dma_is_busy(void);
void adc_dma_irq_handler(void);

#endif // ADC_H
