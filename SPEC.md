# Drone-Arm Firmware Technical Specification

Version: 0.3.2
Target: RP2350B (dual Cortex-M33)  
Toolchain: arm-none-eabi-gcc (cross), host GCC/Python for tests

## Status Legend
- ✅ Complete and Tested
- 🔧 Partial / needs improvement
- ❌ Not yet implemented
- 📝 Spec defined, not implemented

---

## 1. Compilation Pathway ✅

### 1.1 Source-to-Binary Transformation
The compilation pipeline is managed by Meson and Ninja, ensuring a reproducible path from C/ASM source to a flashable UF2 image.

1.  **Preprocessing & Compilation**:
    -   Compiler: `arm-none-eabi-gcc`
    -   Target: Cortex-M33 (`-mcpu=cortex-m33`), Thumb-2 (`-mthumb`)
    -   FPU: Hardware FPv5-D16 (`-mfloat-abi=hard -mfpu=fpv5-d16`)
    -   Flags: `-O2 -ffunction-sections -fdata-sections -fno-builtin`
2.  **Linking**:
    -   Linker Script: `src/linker.ld`
    -   Flags: `-Wl,--gc-sections`, `-nostdlib`, `--specs=nosys.specs`, `--specs=nano.specs`
    -   Result: `firmware.elf`
3.  **Binary Extraction**:
    -   Tool: `arm-none-eabi-objcopy -O binary`
    -   Result: `firmware.bin`
4.  **UF2 Generation**:
    -   Tool: `tools/uf2conv.py`
    -   Magic Start 0: `0x0A324655`
    -   Magic Start 1: `0x9E5D5157`
    -   Magic End: `0x0AB16F30`
    -   Family ID: `0xE48BFF57` (RP2350 ARM-S)
    -   Base Address: `0x10000000` (XIP Flash)
    -   Result: `firmware.uf2`

### 1.2 Boot2 Pipeline
The Stage 2 Bootloader (Boot2) is critical for configuring the QMI interface for Flash access.
1.  Source: `src/boot2.S`
2.  Linker: `src/boot2.ld` (Fixed size 256 bytes)
3.  Post-processing: `tools/pad_checksum`
    -   Pads to 252 bytes with `0xFF`.
    -   Appends 4-byte CRC32 (IEEE) checksum expected by RP2350 ROM.
4.  Assembly Integration: The resulting `boot2_padded.S` is linked into the final `firmware.elf`.

---

## 2. Hardware Validation Guide ✅

### 2.1 Compilation & Flashing Validation
1.  **Build Verification**: Run `ninja -C builddir`. Confirm `firmware.uf2` is generated.
2.  **Artifact Check**: Run `python3 tests/test_build_artifacts.py`. It validates UF2 magics and target addresses.
3.  **Device Detection**: Connect RP2350 in BOOTSEL mode. Run `python3 tools/device_autodetect.py builddir/firmware.uf2`.
4.  **Acceptance**: The device should unmount, reboot, and the application should start.

### 2.2 Functional Validation (Per Module)
-   **GPIO**: Verify heartbeat LED on GPIO 25.
-   **UART**: Verify "Board Init Complete" message at 115200 baud on UART0.
-   **Multicore**: Verify Core1 increments a shared counter via SIO FIFO.

---

## 3. Build System & Standards ✅

### 3.1 Meson Build Configuration
-   Strict enforcement of `-Wall -Wextra -Werror`.
-   Native tests for host-compatible logic (UF2, Scheduler logic).
-   Cross-compilation using `arm-none-eabi.ini`.

### 3.2 NASA Power of Ten Compliance
1.  Simple control flow (no recursion, no `goto`).
2.  Fixed loop bounds (all `while` loops must have a `TIMEOUT`).
3.  No dynamic memory allocation (`malloc/free`) after startup.
4.  Function length limit (60 lines).
5.  Assertion density (minimum 2 assertions per function).
6.  Small data scope.
7.  Check all return values.
8.  Limited preprocessor use.
9.  Pointer limit (max 1 level of dereference).
10. Zero warnings.

---

## 4. Memory Map ✅

Core memory regions (RP2350):
-   **ROM**: `0x00000000` (64KB) - Factory bootloader.
-   **XIP Flash**: `0x10000000` (mapped up to 16MB).
-   **XIP SRAM/Cache**: `0x13FFC000` (16KB) - Flash cache.
-   **Main SRAM**: `0x20000000` (512KB).
-   **Scratch X**: `0x20080000` (4KB).
-   **Scratch Y**: `0x20081000` (4KB).

Peripheral Base Addresses:
-   **SYSINFO**: `0x40000000`
-   **CLOCKS**: `0x40010000`
-   **RESETS**: `0x40020000`
-   **IO_BANK0**: `0x40028000`
-   **PADS_BANK0**: `0x40038000`
-   **XOSC**: `0x40048000`
-   **PLL_SYS**: `0x40050000`
-   **UART0**: `0x40070000`
-   **SPI0**: `0x40080000`
-   **I2C0**: `0x40090000`
-   **DMA**: `0x50000000`
-   **PIO0**: `0x50200000`
-   **SIO**: `0xD0000000`

---

## 5. Startup & Reset Sequence ✅

### 5.1 Reset Handler (`src/startup.S`)
1.  Initialize Stack Pointer from vector table.
2.  **Data Copy**: Copy `.data` from Flash (LMA) to SRAM (VMA).
3.  **BSS Zeroing**: Initialize `.bss` region in SRAM to zero.
4.  **Ramfunc Copy**: Copy `.ramfunc` code to SRAM for low-latency execution.
5.  **Main Entry**: Branch to `main()`.

### 5.2 Board Initialization (`src/board.c`)
1.  **Clocks**: Bring up XOSC and PLL_SYS (150MHz).
2.  **Resets**: Release peripherals from reset.
3.  **GPIO**: Configure pin muxing for UART, I2C, SPI.
4.  **Multicore**: Launch Core1 and perform handshake.

---

## 6. Hardware Drivers

### 6.1 GPIO ✅
-   **Registers**: IO_BANK0 (Mux), PADS_BANK0 (Electrical), SIO (Data).
-   **Functionality**: `gpio_set_function`, `gpio_set_dir`, `gpio_put`, `gpio_get`.
-   **Validation**: Assert `pin < 48`. Toggle LED pin and observe.

### 6.2 UART ✅
-   **Controller**: PL011.
-   **Configuration**: 8N1, FIFO enabled, programmable baudrate.
-   **Validation**: Loopback test on UART0. Verify output on serial terminal.

### 6.3 I2C ✅
-   **Controller**: DW_apb_i2c.
-   **Mode**: Master, Standard/Fast speed.
-   **Validation**: Read WHO_AM_I register from LSM6DSO IMU (Address `0x6A/0x6B`, ID `0x6C`).

### 6.4 SPI ✅
-   **Controller**: PrimeCell SSP (PL022).
-   **Mode**: Master, 8-bit Motorola frame.
-   **Validation**: Read WHO_AM_I from ICM-42670-P (Address `0x67`).

### 6.5 DMA ✅
-   **Channels**: 0-11 (Basic), 12-15 (Lite).
-   **Validation**: Memory-to-memory copy with 32-bit words, verify integrity.

### 6.6 PIO ✅
-   **Blocks**: PIO0, PIO1, PIO2.
-   **Logic**: 32-instruction shared memory, 4 state machines per block.
-   **Validation**: Load a simple square-wave generator program and verify frequency on GPIO.

### 6.7 PWM Output Controller ✅
-   **Interface**: `src/pwm_output.h` / `src/pwm_output.c`.
-   **Signal**: User-defined RC-PWM frequency (e.g. 50 Hz servo, 400 Hz fast-ESC); pulse range and neutral position configurable at init time via `pwm_output_config_t`.
-   **Period**: Derived automatically as `period_us = 1 000 000 / freq_hz`.
-   **Channels**: 1 – 8 (controlled by `num_channels`), each assigned its own GPIO pin.
-   **Functions**: `pwm_output_init`, `pwm_output_set_pulse`, `pwm_output_set_all`, `pwm_output_reset`, `pwm_output_get_pulse`, `pwm_output_get_period_us`, `pwm_output_update`.
-   **Convenience macros**: `PWM_OUTPUT_CFG_ESC_DEFAULTS` (1000–2000 µs, 400 Hz) and `PWM_OUTPUT_CFG_SERVO_DEFAULTS` (500–2500 µs, 50 Hz).
-   **Note**: `src/esc_controller.h` / `src/esc_controller.c` are forwarding stubs kept for reference; all new code should use `pwm_output`.
-   **Validation**: Host test `tests/test_pwm_output.c` – 35 assertions covering ESC and servo configs, user-defined frequencies, clamping, set_all, reset, and uninitialised-safe return values (all passing).

---

## 7. Multicore & Scheduler ✅

### 7.1 Inter-Core Communication
-   **Hardware**: SIO FIFO (8-entry deep).
-   **Spinlocks**: 32 hardware spinlocks for mutual exclusion.

### 7.2 Scheduler Logic ✅
-   **Type**: Cooperative Multicore Scheduler.
-   **Storage**: Static task table, no heap usage.
-   **Policy**: Priority-based round-robin.
-   **Validation**:
    -   *Software*: Host tests in `tests/test_scheduler.c` covering `scheduler_create`, `scheduler_kill`, `scheduler_query`, `scheduler_yield`, `scheduler_sleep`, `scheduler_wait`, `scheduler_run_once`, core affinity, and table-full behaviour (55 assertions, all passing).
    -   *Hardware*: Core0 runs Task A (LED Blink), Core1 runs Task B (UART Heartbeat).

---

## 8. Device Auto-Detection ✅

### 8.1 Discovery Mechanism
-   Detects RP2350 USB Mass Storage Class.
-   Reads `INFO_UF2.TXT` for hardware metadata.
-   **Host tool**: `tools/device_autodetect.py` – retries flashing if the device unmounts prematurely.
-   **Firmware helper**: `src/device_autodetect.c` – `device_was_usb_boot()` / `device_is_app_running()` read the ROM boot-type word from Scratch X SRAM.
-   **Validation**: Host tool retries flashing if the device unmounts prematurely or fails to reboot.

---

## 9. Clock Safety ✅

### 9.1 Internal State Tracking
-   `src/drivers/clock_internal.h` exposes `clock_config_t`, `clock_state_init`, `clock_is_valid`, `clock_get_config`, `clock_set_config`.
-   A magic-number guard detects uninitialised / corrupted clock state.
-   **Validation**: Host test `tests/test_clock_safety.c` verifies init, config update, and post-corruption state (3 tests, all passing).

---

## 10. Glossary
- **XIP**: Execute-In-Place.
- **UF2**: USB Flashing Format.
- **QMI**: Quad SPI Mask Interface.
- **SIO**: Single-cycle IO.
- **P10**: NASA Power of Ten Rules.
