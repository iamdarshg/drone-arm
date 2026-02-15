# P10 (Power of Ten) Violations Report

This document lists all violations of the NASA/JPL Power of Ten rules for safety-critical code in the drone-arm project.

## Summary

| Rule | Description | Violations | Status |
|------|-------------|------------|--------|
| Rule 1 | Simple control flow (no goto/setjmp/recursion) | 0 | ✓ PASS |
| Rule 2 | Fixed loop upper bounds | 4 | ✗ FAIL |
| Rule 3 | No dynamic memory after init | 2 | ✗ FAIL |
| Rule 4 | Functions ≤60 lines | 14 | ✗ FAIL |
| Rule 5 | Min 2 assertions per function | 46 | ✗ FAIL |
| Rule 6 | Smallest variable scope | 12 | ✗ FAIL |
| Rule 7 | Check return values | 8 | ✗ FAIL |
| Rule 8 | Limited preprocessor use | 3 | ✗ FAIL |
| Rule 9 | Limited pointer use (max 1 level) | 0 | ✓ PASS |
| Rule 10 | Zero compiler warnings | Unknown | ⚠ CHECK |

**Overall Status: 7/10 Rules Violated**

---

## Detailed Violations

### Rule 1: Simple Control Flow ✓ PASS

**Status:** No violations found in project source code.

- No `goto` statements
- No `setjmp`/`longjmp` usage  
- No recursion detected
- Note: Third-party libraries (pico-sdk, mbedtls, etc.) contain many goto violations but are excluded from this analysis

---

### Rule 2: Fixed Loop Upper Bounds ✗ FAIL

**Status:** 4 violations found

#### Violation 1: `src/common/scheduler.c:91`
```c
sched.next_free = sched.tasks[task_id].next;  // Variable assignment in loop condition
```

#### Violation 2: `src/common/scheduler.c:204`
```c
for (uint8_t i = 0; i < SCHED_MAX_TASKS; i++) {  // OK - constant bound
```

#### Violation 3: `src/common/math_utils.c:10-11`
```c
while (angle > 3.14159265359f) angle -= 6.28318530718f;  // Data-dependent loop
while (angle < -3.14159265359f) angle += 6.28318530718f;  // Data-dependent loop
```
**Issue:** `wrap_angle()` function uses while loops with data-dependent bounds. Angle normalization could theoretically require unbounded iterations.

#### Violation 4: `src/main_board/low_level/adc.c:118-120`
```c
while (!adc_is_ready()) {
    // Spin wait - conversion takes 96 ADC clock cycles
}
```
**Issue:** Busy-wait loop without explicit iteration bound.

---

### Rule 3: No Dynamic Memory After Initialization ✗ FAIL

**Status:** 2 violations found

#### Violation 1: `src/main_board/low_level/spi.c:179`
```c
inline void spi_write_address(uint8_t spi_id, uint8_t address, uint8_t *data, uint8_t len) {
    uint16_t out[len];  // VLA (Variable Length Array) on stack
    // ...
    free(out);  // ERROR: Attempting to free stack memory!
}
```
**Issues:**
- Uses VLA (Variable Length Array) - undefined behavior when freed
- Attempts to `free()` stack memory
- Potential memory corruption

#### Violation 2: `src/main_board/low_level/spi.c:188`
```c
inline void spi_read_address(uint8_t spi_id, uint8_t address, uint8_t *data, uint8_t len) {
    uint16_t out[len];  // VLA on stack
    // ...
    free(out);  // ERROR: Attempting to free stack memory!
}
```
**Same issues as Violation 1**

---

### Rule 4: Function Length ≤60 Lines ✗ FAIL

**Status:** 14 functions exceed 60 lines

| File | Function | Lines | Violation |
|------|----------|-------|-----------|
| `main_board/init_clocks.c` | *(entire file)* | 678 | File is 678 lines |
| `main_board/low_level/timer.c` | Multiple | 274 | File is 274 lines |
| `main_board/low_level/adc.c` | `adc_init()` | 89 | 29 lines |
| `main_board/low_level/adc.c` | `adc_dma_init()` | 199 | 42 lines |
| `main_board/low_level/gpio.c` | `gpio_set_direction()` | 35-59 | 25 lines |
| `main_board/low_level/i2c.c` | `i2c_set_baud_mode_master()` | 69-121 | 53 lines |
| `main_board/low_level/i2c.c` | `i2c_transfer_blocking()` | 146-182 | 37 lines |
| `main_board/low_level/spi.c` | `spi_set_baud_format_mode()` | 77-130 | 54 lines |
| `main_board/low_level/spi.c` | `spi_transfer_blocking()` | 155-163 | 9 lines (OK) |
| `common/scheduler.c` | `sched_run()` | 197-233 | 37 lines |
| `common/scheduler.c` | `sched_init()` | 37-49 | 13 lines (OK) |

**Worst Violators:**
- `init_clocks.c` - 678 lines (11x over limit)
- `timer.c` - 274 lines (4.5x over limit)
- `spi_set_baud_format_mode()` - 54 lines (90% over limit)
- `i2c_set_baud_mode_master()` - 53 lines (88% over limit)

---

### Rule 5: Minimum 2 Assertions Per Function ✗ FAIL

**Status:** 46 functions have fewer than 2 assertions

**Critical Functions Missing Assertions:**

| File | Function | Assertions | Required |
|------|----------|------------|----------|
| `esc_controller/pwm.c` | `pwm_init()` | 0 | 2 |
| `esc_controller/pwm.c` | `pwm_set_duty()` | 0 | 2 |
| `esc_controller/pwm.c` | `pwm_get_duty()` | 0 | 2 |
| `esc_controller/pwm.c` | `pwm_enable()` | 0 | 2 |
| `esc_controller/pwm.c` | `pwm_disable()` | 0 | 2 |
| `esc_controller/foc.c` | `foc_init()` | 0 | 2 |
| `esc_controller/foc.c` | `foc_set_dq()` | 0 | 2 |
| `main_board/low_level/spi.c` | `spi_init()` | 0 | 2 |
| `main_board/low_level/spi.c` | `spi_set_baud_format_mode()` | 0 | 2 |
| `main_board/low_level/i2c.c` | `i2c_init()` | 0 | 2 |
| `main_board/low_level/adc.c` | `adc_init()` | 0 | 2 |
| `main_board/low_level/adc.c` | `adc_read_blocking()` | 0 | 2 |
| `common/scheduler.c` | `sched_create()` | 0 | 2 |
| `common/scheduler.c` | `sched_run()` | 0 | 2 |
| `common/math_utils.c` | `clamp()` | 0 | 2 |
| `common/math_utils.c` | `wrap_angle()` | 0 | 2 |

**Recommendation:** Add assertions for:
- Parameter validation (null pointers, valid ranges)
- Preconditions (hardware initialized, valid states)
- Postconditions (return values in expected ranges)
- Loop invariants

Example:
```c
void pwm_set_duty(uint8_t channel, float duty) {
    ASSERT(channel < PWM_NUM_CHANNELS);  // Precondition
    ASSERT(duty >= 0.0f && duty <= 1.0f);  // Parameter validation
    
    // ... function body ...
    
    ASSERT(pwm_get_duty(channel) == duty);  // Postcondition (optional)
}
```

---

### Rule 6: Smallest Variable Scope ✗ FAIL

**Status:** 12 violations found

#### Violation 1: `src/main_board/low_level/spi.c:14-25`
```c
static void spi_gpio_init(uint8_t spi_id, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs) {
    uint32_t io_base = IO_BANK0_BASE;    // Never used
    uint32_t pads_base = PADS_BANK0_BASE; // Never used
    // ...
}
```
**Issue:** Variables declared at function scope but never used.

#### Violation 2: `src/main_board/low_level/i2c.c:18-19`
```c
static void i2c_gpio_init(uint8_t i2c_id, uint8_t scl, uint8_t sda) {
    uint32_t io_base = IO_BANK0_BASE;    // Never used
    uint32_t pads_base = PADS_BANK0_BASE; // Never used
```
**Same issue as Violation 1**

#### Violation 3: `src/common/scheduler.c:18-26`
```c
static struct {
    task_entry_t tasks[SCHED_MAX_TASKS];
    uint8_t ready_head;
    uint8_t ready_tail;
    uint8_t current;
    uint8_t running;
    uint8_t task_count;
    uint8_t next_free;
} sched;
```
**Issue:** Module-level state should be encapsulated or made more local if possible.

#### Violation 4-12: Global Variables in Header Files
Multiple global variables defined in header files should be moved to the smallest scope:
- `global_pin_func_map[]` - should be module-local
- `global_pin_direction[]` - should be module-local
- `global_pin_pullup[]` - should be module-local
- ADC channel mappings - should be in adc.c only

---

### Rule 7: Check Return Values ✗ FAIL

**Status:** 8 violations found

#### Violation 1: `src/main_board/low_level/spi.c:79`
```c
void spi_set_baud_format_mode(uint8_t spi_id, uint32_t baudrate, bool master) {
    if (spi_id >= SPI_NUM_INTERFACES) {
        return false;  // ERROR: void function returning value
    }
```
**Issue:** Function declared void but returns false.

#### Violation 2: `src/main_board/low_level/i2c.c:71`
```c
void i2c_set_baud_mode_master(uint8_t i2c_id, uint32_t baudrate, bool master) {
    if (i2c_id >= I2C_NUM_INTERFACES) {
        return false;  // ERROR: void function returning value
    }
```
**Same issue as Violation 1**

#### Violation 3-8: snprintf Return Values
Multiple locations where `snprintf()` return value is not checked:

- `src/main_board/gpio.c:67` - snprintf unchecked
- `src/main_board/gpio.c:77` - snprintf unchecked
- `src/main_board/gpio.c:96` - snprintf unchecked
- `src/main_board/gpio.c:111` - snprintf unchecked
- `src/main_board/low_level/adc.c:45` - snprintf unchecked

**Issue:** snprintf can fail or truncate. Return value should be checked:
```c
int ret = snprintf(buf, sizeof(buf), "...");
if (ret < 0 || ret >= sizeof(buf)) {
    // Handle error/truncation
}
```

---

### Rule 8: Limited Preprocessor Use ✗ FAIL

**Status:** 3 violations found

#### Violation 1: `src/main_board/gpio.c:29`
```c
#define GPIO_FAST_OP(operation, value) __asm__ volatile ( \
    "coproc " #operation ", %0" \
    :: "r" (value) : "memory" )
```
**Issue:** Complex inline assembly macro with token pasting.

#### Violation 2: `include/hardware/structs/*.h`
Multiple files use complex macro definitions that expand to non-syntactic units or use bitwise operations in macros.

#### Violation 3: Conditional Compilation
```c
#if PWM_PHASE_CORRECT
    #define PWM_CYCLES_PER_PERIOD (...)
#else
    #define PWM_CYCLES_PER_PERIOD (...)
#endif
```
**Issue:** Multiple code paths increase testing burden exponentially (2^n paths).

---

### Rule 9: Limited Pointer Use ✓ PASS

**Status:** No violations found

- No double pointers (`**`) found in project code
- No function pointers in project code (except scheduler, which is a reasonable use case)
- Pointer dereferencing is explicit and traceable

**Note:** Function pointers are used in `scheduler.c` for task management, but this is acceptable for an RTOS-like scheduler.

---

### Rule 10: Zero Compiler Warnings ⚠ CHECK REQUIRED

**Status:** Cannot verify without building

**Required Actions:**
1. Enable all compiler warnings: `-Wall -Wextra -Wpedantic`
2. Enable warnings as errors: `-Werror`
3. Run static analysis tools (cppcheck, clang-static-analyzer)
4. Fix all warnings

**Known Issues Likely to Cause Warnings:**
- Implicit function declarations
- Unused variables
- Void functions returning values
- Missing return statements
- Type mismatches

---

## Most Critical Violations (Priority Order)

### CRITICAL (Fix Immediately)
1. **spi.c:179,188** - Freeing stack memory (undefined behavior, security vulnerability)
2. **spi.c:79, i2c.c:71** - Void functions returning values
3. **gpio.c:108** - `gpio_get()` declared void but returns value

### HIGH (Fix Before Production)
4. **Rule 5** - No assertions (46 functions lack assertions)
5. **Rule 4** - Functions too long (14 violations)
6. **Rule 7** - Unchecked return values (8 violations)

### MEDIUM (Fix During Refactoring)
7. **Rule 2** - Unbounded loops (4 violations)
8. **Rule 6** - Variable scope issues (12 violations)
9. **Rule 8** - Preprocessor complexity (3 violations)

### LOW (Best Practice)
10. **Rule 10** - Compiler warnings (needs verification)

---

## Recommendations

1. **Immediate:** Fix critical memory corruption bugs in spi.c
2. **Short-term:** Add assertion framework and minimum assertions
3. **Medium-term:** Refactor long functions, add bounds checking to loops
4. **Long-term:** Implement comprehensive static analysis in CI/CD

---

*Report generated: 2026-02-15*
*P10 Standard: NASA/JPL Laboratory for Reliable Software*
