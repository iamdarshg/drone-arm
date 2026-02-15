
#ifndef ERRORS_H
#define ERRORS_H

static inline void log_error(const char *message, int severity, const char *function) {
    (void)message;
    (void)severity;
    (void)function;
}
#endif
// severity levels- 
// 0: Info
// 1: initialization error
// 2: function call error (e.g., calling a function with invalid parameters or in an invalid state)
