/**
 * ADC Driver for Main Controller (RP2350B)
 * P10 Compliant - Fixed error handling and assertions
 */

#include "adc.h"
#include <stdio.h>
#include "../hardware_defs/pins.h"
#include "../../common/assert.h"
#include "../../common/errors.h"

// Static helper for snprintf errors
static void log_snprintf_error(const char* func) {
    log_error("snprintf buffer truncation", 1, func);
}

// ============================================================================
// ADC to GPIO Channel Mapping for RP2350
// ============================================================================

static const uint8_t adc_gpio_to_channel[48] = {
    0, 1, 2, 3,       // GPIO0-3 (not bonded on some packages)
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO4-7
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO8-11
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO12-15
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO16-19
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO20-23
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO24-27
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO28-31
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO32-35
    0xFF, 0xFF, 0xFF, 0xFF, // GPIO36-39
    1, 0xFF, 3, 0xFF,       // GPIO40-43 (ADC1, ADC3)
    0xFF, 5, 0xFF, 7        // GPIO44-47 (ADC5, ADC7)
};

// DMA channel base addresses for fast access
#define DMA_CH_BASE         (DMA_BASE + 0x40 * ADC_DMA_CHANNEL)
#define DMA_CH_READ_ADDR    (DMA_CH_BASE + DMA_CH0_READ_ADDR_OFFSET)
#define DMA_CH_WRITE_ADDR   (DMA_CH_BASE + DMA_CH0_WRITE_ADDR_OFFSET)
#define DMA_CH_TRANS_COUNT  (DMA_CH_BASE + DMA_CH0_TRANS_COUNT_OFFSET)
#define DMA_CH_CTRL_TRIG    (DMA_CH_BASE + DMA_CH0_CTRL_TRIG_OFFSET)

// ============================================================================
// Initialization
// ============================================================================

void adc_init_gpio(uint8_t gpio) {
    PRECONDITION(gpio < 48);
    
    adc_disable();
    
    uint8_t channel = adc_gpio_to_channel[gpio];
    if (channel == 0xFF) {
        char error_msg[64];
        int ret = snprintf(error_msg, sizeof(error_msg), 
                          "GPIO %d is not a valid ADC pin", gpio);
        if (ret < 0 || (size_t)ret >= sizeof(error_msg)) {
            log_snprintf_error("adc_init_gpio");
        }
        log_error(error_msg, 2, "adc_init_gpio");
        return;
    }
    
    uint32_t ctrl_addr = IO_BANK0_BASE + 4 + (gpio * 8);
    REG32_WRITE(ctrl_addr, GPIO_FUNC_NULL);
    
    uint32_t pad_addr = PADS_BANK0_BASE + 4 + (gpio * 4);
    REG32_WRITE(pad_addr, 0);
}

void adc_init(void) {
    adc_init_gpio(ADC_CHANNEL_1_PIN);
    adc_init_gpio(ADC_CHANNEL_3_PIN);
    adc_init_gpio(ADC_CHANNEL_5_PIN);
    adc_init_gpio(ADC_CHANNEL_7_PIN);
    
    adc_enable();
    
    // Wait for ADC ready with timeout (Rule 2: fixed bound)
    uint32_t timeout = 10000;
    while (!adc_is_ready()) {
        ASSERT_TERMINATION(timeout--, 10001);
        if (timeout == 0) {
            log_error("ADC ready timeout", 2, "adc_init");
            return;
        }
    }
    
    adc_set_clkdiv(ADC_DIV_INT, ADC_DIV_FRAC);
    adc_fifo_set_threshold(1);
    adc_fifo_set_shift(false);
    adc_fifo_enable_dma_req();
    adc_fifo_clear_overflow();
    adc_fifo_clear_underflow();
    adc_fifo_enable();
    
    adc_set_round_robin_mask(ADC_RROBIN_MASK);
}

void adc_set_round_robin(bool enable) {
    if (enable) {
        adc_set_round_robin_mask(ADC_RROBIN_MASK);
        adc_start_many();
    } else {
        adc_stop_many();
    }
}

// ============================================================================
// Blocking Read
// ============================================================================

uint16_t adc_read_blocking(uint8_t channel) {
    PRECONDITION(channel < 8);  // RP2350 has 8 ADC channels
    
    adc_stop_many();
    adc_set_round_robin_mask(0);
    adc_select_channel(channel);
    adc_start_once();
    
    // Wait for completion with timeout (Rule 2: fixed bound)
    uint32_t timeout = 10000;
    while (!adc_is_ready()) {
        ASSERT_TERMINATION(timeout--, 10001);
        if (timeout == 0) {
            log_error("ADC read timeout", 2, "adc_read_blocking");
            return 0;
        }
    }
    
    return adc_get_result();
}

// ============================================================================
// Voltage Conversion
// ============================================================================

float adc_convert_to_voltage(uint16_t adc_value) {
    ASSERT(adc_value <= 4095);  // 12-bit ADC
    return ((float)adc_value * 3.3f) / 4095.0f;
}

float adc_read_temperature(void) {
    adc_temp_sensor_enable();
    
    uint16_t raw = adc_read_blocking(ADC_TEMP_CHANNEL);
    
    adc_temp_sensor_disable();
    
    float voltage = adc_convert_to_voltage(raw);
    return 25.0f - (voltage - 0.706f) / 0.001721f;
}

// ============================================================================
// DMA Support
// ============================================================================

static uint16_t *adc_dma_buffer = NULL;
static uint32_t adc_dma_samples = 0;
static volatile bool adc_dma_busy = false;

void adc_dma_init(uint16_t *buffer, uint32_t num_samples) {
    PRECONDITION(buffer != NULL);
    ASSERT(num_samples > 0);
    ASSERT(num_samples <= 1024);  // Reasonable upper limit
    
    adc_dma_buffer = buffer;
    adc_dma_samples = num_samples;
    
    dma_hw->ch[ADC_DMA_CHANNEL].ctrl_trig = 0;
    dma_hw->ch[ADC_DMA_CHANNEL].read_addr = (uint32_t)&adc_hw->fifo;
    dma_hw->ch[ADC_DMA_CHANNEL].write_addr = (uint32_t)buffer;
    dma_hw->ch[ADC_DMA_CHANNEL].transfer_count = num_samples;
    
    uint32_t ctrl = DMA_CTRL_EN_BIT | 
                    DMA_CTRL_HIGH_PRIORITY |
                    DMA_CTRL_DATA_SIZE_HALF |
                    DMA_CTRL_INCR_WRITE |
                    (DMA_TREQ_ADC << 17);
    
    dma_hw->ch[ADC_DMA_CHANNEL].al1_ctrl = ctrl;
}

void adc_dma_start(void) {
    PRECONDITION(adc_dma_buffer != NULL);
    
    dma_hw->ch[ADC_DMA_CHANNEL].write_addr = (uint32_t)adc_dma_buffer;
    dma_hw->ch[ADC_DMA_CHANNEL].transfer_count = adc_dma_samples;
    dma_hw->intr = 1U << ADC_DMA_CHANNEL;
    dma_hw->ch[ADC_DMA_CHANNEL].ctrl_trig = 
        dma_hw->ch[ADC_DMA_CHANNEL].al1_ctrl | DMA_CTRL_EN_BIT;
    
    adc_set_round_robin(true);
    adc_dma_busy = true;
}

void adc_dma_stop(void) {
    adc_stop_many();
    dma_hw->ch[ADC_DMA_CHANNEL].ctrl_trig = 0;
    adc_dma_busy = false;
}

bool adc_dma_is_busy(void) {
    if (!adc_dma_busy) {
        return false;
    }
    
    bool busy = (dma_hw->ch[ADC_DMA_CHANNEL].ctrl_trig & (1U << 26)) != 0;
    
    if (!busy) {
        adc_dma_busy = false;
    }
    
    return busy;
}

void adc_dma_irq_handler(void) {
    if (dma_hw->ints & (1U << ADC_DMA_CHANNEL)) {
        dma_hw->intr = 1U << ADC_DMA_CHANNEL;
        adc_dma_stop();
        adc_dma_busy = false;
    }
}
