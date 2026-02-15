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

#endif // SYS_H
