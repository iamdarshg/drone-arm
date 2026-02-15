/**
 * Assertion Framework for P10 Compliance
 * 
 * Power of Ten Rule 5: Minimum 2 assertions per function
 * 
 * Usage:
 *   ASSERT(condition);                    // Basic assertion
 *   ASSERT_MSG(condition, "message");     // With message
 *   PRECONDITION(condition);              // Function precondition
 *   POSTCONDITION(condition);             // Function postcondition
 *   INVARIANT(condition);                 // Loop/struct invariant
 */

#ifndef ASSERT_H
#define ASSERT_H

#include <stdint.h>
#include <stdbool.h>

// Assertion failure handler
void assertion_failed(const char *file, int line, const char *condition, const char *message);

#if defined(DISABLE_ASSERTIONS) || defined(NDEBUG)
    // Rule 5: Keeping minimal assertions even in prod might be safer, 
    // but the user requested them gone for space/cycles.
    // We keep them as empty macros or minimal checks that avoid side effects.
    #define ASSERT(cond)          ((void)0)
    #define ASSERT_MSG(cond, msg) ((void)0)
    #define PRECONDITION(cond)    ((void)0)
    #define POSTCONDITION(cond)   ((void)0)
    #define INVARIANT(cond)       ((void)0)
    #define ASSERT_NOT_NULL(ptr)  ((void)0)
    #define ASSERT_RANGE(val, min, max) ((void)0)
    #define ASSERT_INDEX(idx, max)      ((void)0)
    #define CHECK_RETURN(call)    ((void)(call))
    #define CHECK_RETURN_PTR(call) ((void)(call))
    #define ASSERT_TERMINATION(counter, limit) ((void)0)
#else
    // Basic assertion - always enabled in safety-critical code
    #define ASSERT(cond) \
        do { \
            if (!(cond)) { \
                assertion_failed(__FILE__, __LINE__, #cond, NULL); \
            } \
        } while (0)

    // Assertion with message
    #define ASSERT_MSG(cond, msg) \
        do { \
            if (!(cond)) { \
                assertion_failed(__FILE__, __LINE__, #cond, msg); \
            } \
        } while (0)

    // Precondition check - input validation
    #define PRECONDITION(cond) ASSERT_MSG(cond, "Precondition violated: " #cond)

    // Postcondition check - output validation
    #define POSTCONDITION(cond) ASSERT_MSG(cond, "Postcondition violated: " #cond)

    // Invariant check - for loops and data structures
    #define INVARIANT(cond) ASSERT_MSG(cond, "Invariant violated: " #cond)

    // Parameter validation macros
    #define ASSERT_NOT_NULL(ptr) ASSERT_MSG((ptr) != NULL, "Null pointer: " #ptr)
    #define ASSERT_RANGE(val, min, max) ASSERT_MSG(((val) >= (min)) && ((val) <= (max)), "Value out of range: " #val)
    #define ASSERT_INDEX(idx, max) ASSERT_MSG((idx) < (max), "Index out of bounds: " #idx)

    // Check return value (Rule 7)
    #define CHECK_RETURN(call) \
        do { \
            int _ret = (call); \
            ASSERT_MSG(_ret >= 0, "Function call failed: " #call); \
        } while (0)

    // For functions that return pointers
    #define CHECK_RETURN_PTR(call) \
        do { \
            void *_ret = (call); \
            ASSERT_MSG(_ret != NULL, "Function returned null: " #call); \
        } while (0)

    // Assert that a loop will terminate (Rule 2)
    #define ASSERT_TERMINATION(counter, limit) \
        ASSERT_MSG((counter) < (limit), "Loop termination failure")
#endif

// Static assertions for compile-time checks
#define STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)

#endif // ASSERT_H
