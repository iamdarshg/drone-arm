#ifndef SYS_H
#define SYS_H

/** @brief Initialize Nested Vectored Interrupt Controller. */
void init_nvic(void);
/** @brief Initialize Glitch Detector. */
void init_glitch_detector(void);
/** @brief Initialize Watchdog timer. */
void init_watchdog(void);
/** @brief Initialize Task Scheduler. */
void init_scheduler(void);

void global_irq_enable(void);
void global_irq_disable(void);


#endif // SYS_H
