#include "ring_buffer.h"
#include <stdbool.h>

void ring_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t size) {
    rb->buffer = buffer; rb->size = size; rb->head = rb->tail = 0;
}
uint16_t ring_write(ring_buffer_t *rb, const uint8_t *data, uint16_t len) {
    uint16_t i; for (i = 0; i < len && !ring_full(rb); i++) {
        rb->buffer[rb->head] = data[i];
        rb->head = (rb->head + 1) % rb->size;
    } return i;
}
uint16_t ring_read(ring_buffer_t *rb, uint8_t *data, uint16_t len) {
    uint16_t i; for (i = 0; i < len && !ring_empty(rb); i++) {
        data[i] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % rb->size;
    } return i;
}
uint16_t ring_available(ring_buffer_t *rb) {
    return (rb->head >= rb->tail) ? (rb->head - rb->tail) : (rb->size - rb->tail + rb->head);
}
bool ring_empty(ring_buffer_t *rb) { return rb->head == rb->tail; }
bool ring_full(ring_buffer_t *rb) { return ((rb->head + 1) % rb->size) == rb->tail; }
