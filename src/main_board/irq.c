#include <stdint.h>
#include <stdbool.h>

typedef void (*irq_handler_t)(void);

void irq_init(void);
void irq_enable(uint8_t irq_num);
void irq_disable(uint8_t irq_num);
void irq_set_handler(uint8_t irq_num, irq_handler_t handler);
void irq_set_priority(uint8_t irq_num, uint8_t priority);
