#include "adc.h"
#include <stdio.h>
#include "hardware_defs/pins.h"
#include "common/errors.h"
#include "common/utils.h"

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
    // Disable ADC to allow pin configuration
    adc_disable();
    
    // Verify this is a valid ADC pin
    uint8_t channel = adc_gpio_to_channel[gpio];
    if (channel == 0xFF) {
        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg), "GPIO %d is not a valid ADC pin", gpio);
        log_error(error_msg, 2, "adc_init_gpio");
        return;
    }
    
    // Configure pin function to NULL (analog input)
    uint32_t ctrl_addr = IO_BANK0_BASE + 4 + (gpio * 8);
    REG32_WRITE(ctrl_addr, GPIO_FUNC_NULL);
    
    // Disable digital input buffer, enable analog
    uint32_t pad_addr = PADS_BANK0_BASE + 4 + (gpio * 4);
    REG32_WRITE(pad_addr, 0);  // Disable input, no pull-up/down
}

void adc_init(void) {
    // Initialize all active ADC GPIOs
    adc_init_gpio(ADC_CHANNEL_1_PIN);
    adc_init_gpio(ADC_CHANNEL_3_PIN);
    adc_init_gpio(ADC_CHANNEL_5_PIN);
    adc_init_gpio(ADC_CHANNEL_7_PIN);
    
    // Power on ADC
    adc_enable();
    
    // Wait for ADC ready (typically immediate, but datasheet recommends)
    while (!adc_is_ready()) {
        // Spin wait - usually single cycle
    }
    
    // Set clock divider for target sample rate
    adc_set_clkdiv(ADC_DIV_INT, ADC_DIV_FRAC);
    
    // Configure FIFO for DMA
    adc_fifo_set_threshold(1);  // Trigger DMA when 1+ samples ready
    adc_fifo_set_shift(false);  // Keep full 12-bit results
    adc_fifo_enable_dma_req();
    adc_fifo_clear_overflow();
    adc_fifo_clear_underflow();
    
    // Enable FIFO
    adc_fifo_enable();
    
    // Setup round-robin for all active channels
    adc_set_round_robin_mask(ADC_RROBIN_MASK);
}

void adc_set_round_robin(bool enable) {
    if (enable) {
        // Start continuous conversions with round-robin
        adc_set_round_robin_mask(ADC_RROBIN_MASK);
        adc_start_many();
    } else {
        // Stop continuous conversions
        adc_stop_many();
    }
}

// ============================================================================
// Blocking Read
// ============================================================================

uint16_t adc_read_blocking(uint8_t channel) {
    // Ensure round-robin is disabled for single reads
    adc_stop_many();
    adc_set_round_robin_mask(0);
    
    // Select channel
    adc_select_channel(channel);
    
    // Start conversion
    adc_start_once();
    
    // Wait for completion - busy poll for minimum latency
    while (!adc_is_ready()) {
        // Spin wait - conversion takes 96 ADC clock cycles
    }
    
    return adc_get_result();
}

// ============================================================================
// Voltage Conversion
// ============================================================================

float adc_convert_to_voltage(uint16_t adc_value) {
    // 12-bit ADC, 3.3V reference
    // V = (ADC / 4095) * 3.3
    return ((float)adc_value * 3.3f) / 4095.0f;
}

float adc_read_temperature(void) {
    // Enable temperature sensor
    adc_temp_sensor_enable();
    
    // Read temperature channel (ADC4)
    uint16_t raw = adc_read_blocking(ADC_TEMP_CHANNEL);
    
    // Disable temperature sensor to save power
    adc_temp_sensor_disable();
    
    // Convert to temperature
    // RP2350 datasheet: V = 0.706V + (T - 25) * -1.721mV
    // Rearranged: T = 25 - (V - 0.706) / 0.001721
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
    if (buffer == NULL || num_samples == 0) {
        log_error("Invalid DMA buffer parameters", 2, "adc_dma_init");
        return;
    }
    
    adc_dma_buffer = buffer;
    adc_dma_samples = num_samples;
    
    // Configure DMA channel
    // Read from ADC FIFO, write to buffer
    // 16-bit transfers, increment write address
    
    // Disable channel first
    dma_hw->ch[ADC_DMA_CHANNEL].ctrl_trig = 0;
    
    // Set read address (ADC FIFO)
    dma_hw->ch[ADC_DMA_CHANNEL].read_addr = (uint32_t)&adc_hw->fifo;
    
    // Set write address (buffer start)
    dma_hw->ch[ADC_DMA_CHANNEL].write_addr = (uint32_t)buffer;
    
    // Set transfer count
    dma_hw->ch[ADC_DMA_CHANNEL].transfer_count = num_samples;
    
    // Configure control register
    // Data size: 16-bit (halfword)
    // Increment write, don't increment read
    // Use DREQ pacing from ADC
    // High priority
    uint32_t ctrl = DMA_CTRL_EN_BIT | 
                    DMA_CTRL_HIGH_PRIORITY |
                    DMA_CTRL_DATA_SIZE_HALF |
                    DMA_CTRL_INCR_WRITE |
                    (DMA_TREQ_ADC << 17);  // TREQ_SEL = ADC
    
    // Don't enable yet - will enable in adc_dma_start()
    dma_hw->ch[ADC_DMA_CHANNEL].al1_ctrl = ctrl;
}

void adc_dma_start(void) {
    if (adc_dma_buffer == NULL) {
        log_error("DMA not initialized", 2, "adc_dma_start");
        return;
    }
    
    // Reset buffer pointer
    dma_hw->ch[ADC_DMA_CHANNEL].write_addr = (uint32_t)adc_dma_buffer;
    dma_hw->ch[ADC_DMA_CHANNEL].transfer_count = adc_dma_samples;
    
    // Clear interrupts
    dma_hw->intr = 1U << ADC_DMA_CHANNEL;
    
    // Enable DMA channel
    dma_hw->ch[ADC_DMA_CHANNEL].ctrl_trig = 
        dma_hw->ch[ADC_DMA_CHANNEL].al1_ctrl | DMA_CTRL_EN_BIT;
    
    // Start ADC conversions with round-robin
    adc_set_round_robin(true);
    
    adc_dma_busy = true;
}

void adc_dma_stop(void) {
    // Stop ADC conversions
    adc_stop_many();
    
    // Disable DMA channel
    dma_hw->ch[ADC_DMA_CHANNEL].ctrl_trig = 0;
    
    adc_dma_busy = false;
}

bool adc_dma_is_busy(void) {
    if (!adc_dma_busy) {
        return false;
    }
    
    // Check if DMA channel is still busy
    bool busy = (dma_hw->ch[ADC_DMA_CHANNEL].ctrl_trig & (1U << 26)) != 0;
    
    if (!busy) {
        adc_dma_busy = false;
    }
    
    return busy;
}

void adc_dma_irq_handler(void) {
    // Check if this is our DMA channel
    if (dma_hw->ints & (1U << ADC_DMA_CHANNEL)) {
        // Clear interrupt
        dma_hw->intr = 1U << ADC_DMA_CHANNEL;
        
        // Stop conversions
        adc_dma_stop();
        
        // Could set a flag or call a callback here
        // For now, just mark as complete
        adc_dma_busy = false;
    }
}
