#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
} ring_buffer_t;

void ring_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t size);
uint16_t ring_write(ring_buffer_t *rb, const uint8_t *data, uint16_t len);
uint16_t ring_read(ring_buffer_t *rb, uint8_t *data, uint16_t len);
uint16_t ring_available(ring_buffer_t *rb);
bool ring_empty(ring_buffer_t *rb);
bool ring_full(ring_buffer_t *rb);
