#ifndef ASSERT_H
#define ASSERT_H

#include <stdint.h>

void assertion_failed(const char *file, int line, const char *cond, const char *msg);

#ifdef NDEBUG
#define ASSERT(cond) ((void)0)
#define ASSERT_MSG(cond, msg) ((void)0)
#define PRECONDITION(cond) ((void)0)
#define POSTCONDITION(cond) ((void)0)
#define INVARIANT(cond) ((void)0)
#define PRECONDITION_NOT_NULL(ptr) ((void)0)
#define PRECONDITION_RANGE(val, lo, hi) ((void)0)
#define ASSERT_INDEX(idx, max) ((void)0)
#define ASSERT_TERMINATION(counter, limit) ((void)0)
#else
#define ASSERT(cond) do { if (!(cond)) assertion_failed(__FILE__, __LINE__, #cond, 0); } while (0)
#define ASSERT_MSG(cond, msg) do { if (!(cond)) assertion_failed(__FILE__, __LINE__, #cond, (msg)); } while (0)
#define PRECONDITION(cond) ASSERT_MSG((cond), "precondition")
#define POSTCONDITION(cond) ASSERT_MSG((cond), "postcondition")
#define INVARIANT(cond) ASSERT_MSG((cond), "invariant")
#define PRECONDITION_NOT_NULL(ptr) ASSERT_MSG((ptr) != 0, "null pointer")
#define PRECONDITION_RANGE(val, lo, hi) ASSERT_MSG(((val) >= (lo)) && ((val) <= (hi)), "out of range")
#define ASSERT_INDEX(idx, max) ASSERT_MSG((idx) < (max), "index out of range")
#define ASSERT_TERMINATION(counter, limit) ASSERT_MSG((counter) < (limit), "loop bound exceeded")
#endif

#define STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)

#endif
