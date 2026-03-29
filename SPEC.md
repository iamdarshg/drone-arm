# Drone-Arm Firmware Technical Specification

Version: 0.3.1
Target: RP2350B (dual Cortex-M33)  
Toolchain: arm-none-eabi-gcc (cross), host GCC/Python for tests

## Status Legend
- ✅ Complete and Tested
- 🔧 Partial / In Progress
- ❌ Not yet implemented
- 📝 Spec defined, not implemented

---

## 1. Compilation Pipeline & Hardware Validation ✅

### 1.1 Transformation Flow
The pipeline transforms C and Assembly source code into a bootable UF2 binary:
1.  **Preprocessing & Compilation**: `arm-none-eabi-gcc` compiles `.c` and `.S` files into ELF object files using `-mcpu=cortex-m33 -mthumb`.
2.  **Linking**: `arm-none-eabi-ld` (via the compiler driver) links objects using `src/linker.ld`. This stage resolves symbols and places sections into their designated memory regions (Flash vs RAM).
3.  **Binary Extraction**: `arm-none-eabi-objcopy` extracts the raw binary from the ELF (`firmware.bin`).
4.  **UF2 Conversion**: A custom Python tool (`tools/uf2conv.py`) wraps the binary into the UF2 format.
    -   **Magic Values**: `start0=0x0A324655`, `start1=0x9E5D5157`, `end=0x0AB16F30`.
    -   **Family ID**: `0xE48BFF57` (RP2350 ARM).
    -   **Target Address**: `0x10000000` (XIP Flash window).

### 1.2 Boot Stage 2 (Boot2) Pipeline
RP2350 requires a checksummed 256-byte boot block at the start of flash:
1.  Compile `src/boot2.S` and link with `src/boot2.ld`.
2.  Extract binary and pass to `tools/pad_checksum`.
3.  The tool calculates a CRC32 (IEEE) over the first 252 bytes and appends it.
4.  The result is emitted as `boot2_padded.S` and linked into the main firmware.

### 1.3 Hardware Validation Guide for Compilation
To validate the pathway on physical hardware:
1.  **Build Verification**: Ensure `firmware.uf2` is generated without errors.
2.  **Flashing**: Connect RP2350 in BOOTSEL mode. Copy `firmware.uf2` to the mounted drive.
3.  **Acceptance Check**:
    -   If the drive unmounts and the board reboots, the ROM accepted the UF2 header and magic numbers.
    -   If a heartbeat LED (configured on GPIO 25 or similar) starts blinking, the Boot2 and Startup sequence are functional.
4.  **Metadata Verification**: Use `picotool info -a firmware.uf2` to verify the Family ID and Absolute Address.

---

## 2. Build System ✅

### 2.1 Meson Configuration
The build system is Meson-based for efficiency and reproducibility.
-   **Compiler Flags**: `-O2 -ffunction-sections -fdata-sections -fno-builtin` for performance and dead-code elimination.
-   **Warnings**: `-Wall -Wextra -Werror` strictly enforced.
-   **Linker Flags**: `-Wl,--gc-sections` to strip unused code.
-   **Optimization**: Aggressive use of hardware-specific instructions via `-mcpu=cortex-m33`.

---

## 3. Memory Map & Startup Sequence ✅

### 3.1 Memory Map
Core memory regions used by this design:
-   **ROM**: `0x00000000` (64KB)
-   **XIP Flash Window**: `0x10000000` (mapped via XIP)
-   **XIP SRAM/Cache Alias**: `0x13FFC000` (16KB)
-   **Main SRAM**: `0x20000000` (512KB)
-   **Scratch X**: `0x20080000` (4KB)
-   **Scratch Y**: `0x20081000` (4KB)

Primary peripheral spaces:
-   **APB region**: starts at `0x40000000`
-   **DMA**: `0x50000000`
-   **PIO blocks**: `0x50200000`, `0x50300000`, `0x50400000`
-   **SIO**: `0xD0000000` (Single-cycle IO)

### 3.2 Boot Process
1.  **ROM Execution**: Validates Boot2 checksum.
2.  **Boot2**: Configures QMI for high-speed flash access.
3.  **Reset Vector**: `src/startup.S` sets up Stack Pointer and jumps to `reset_handler`.
4.  **Runtime Init**:
    -   Copy `.data` from Flash to RAM.
    -   Zero `.bss`.
    -   Initialize `.ramfunc`.
    -   Branch to `main()`.

---

## 4. Hardware Abstraction & Drivers

### 4.1 GPIO ✅
-   **Configuration**: Control IO_BANK0 for function muxing and PADS_BANK0 for electrical traits.
-   **Modes**: Input, Output, Alt (SPI/I2C/UART/PWM).
-   **Interrupts**: Support for level and edge-triggered IRQs.
-   **Policy**: Explicit pin-range checks (`pin < 48`) and direction configuration.
-   **Validation**:
    -   *Software*: Verify `gpio_put` and `gpio_get` correctly manipulate SIO registers.
    -   *Hardware*: Toggle pin 25 (LED) and verify with Oscilloscope/Visual.

### 4.2 I2C ✅
-   **Modes**: Controller (Master) only for this pass. DW APB I2C style programming model.
-   **Addressing**: 7-bit and 10-bit support.
-   **Timing**: Standard (100kbps) and Fast (400kbps) modes.
-   **Validation**:
    -   *Software*: Verify `wait_status` timeouts and ACK/NACK handling.
    -   *Hardware*: Scan bus and read WHO_AM_I register (`0x6C`) from LSM6DSO.

### 4.3 SPI ✅
-   **Modes**: Master, 8-bit Motorola frame format.
-   **Clocking**: Configurable divisor based on `clk_peri`.
-   **Validation**:
    -   *Software*: Loopback test (MISO tied to MOSI).
    -   *Hardware*: Read ID (`0x67`) from ICM-42670-P.

### 4.4 UART ✅
-   **Config**: PL011 standard, 8N1, FIFO enabled.
-   **Validation**:
    -   *Software*: Verify `uart_putc` blocks until FIFO space is available.
    -   *Hardware*: Verify output on serial console at 115200 baud.

### 4.5 DMA ✅
-   **Channel Management**: Static allocation of channels 0-15.
-   **Policy**: Polled completion for this pass.
-   **Validation**:
    -   *Software*: Memory-to-memory block copy comparison of 1KB buffers.
    -   *Hardware*: UART TX triggered by DMA.

### 4.6 PIO ✅
-   **Instruction Format**: 16-bit instructions for state machines. 32-word instruction memory.
-   **Validation**:
    -   *Software*: Verify instruction memory writes via `pio_load_program`.
    -   *Hardware*: Generate a 1MHz square wave on a spare GPIO using a PIO program.

### 4.7 SIO ✅
-   **Features**: Atomic bit-set/clr/tgl for GPIO, hardware spinlocks, and FIFO for inter-core comms.

---

## 5. Multicore & Scheduler ✅

### 5.1 Multicore Initialization
-   **Launch**: Core0 pushes entry point and stack pointer to SIO FIFO.
-   **Handshake**: Synchronized boot sequence using FIFO status flags. Core1 launch handshake must complete before either core enters its scheduler loop.

### 5.2 RTOS-like Scheduler
-   **Type**: Cooperative, multicore-aware.
-   **Task Storage**: Fixed-size static array (MAX_TASKS=16). No heap.
-   **Policies**: Round-robin per core.
-   **Primitives**: Create, Kill, Query, Sleep, Yield, Wait.
-   **Validation**:
    -   *Software*: Host tests for task creation, scheduling, and state transitions.
    -   *Hardware*: Two tasks on different cores incrementing a shared atomic counter or toggling separate LEDs.

---

## 6. Device Auto-Detection ✅

### 6.1 Host-side Tool (`tools/device_autodetect.py`)
-   **Discovery**: Monitor USB/lsblk for Mass Storage devices with label 'RPI-RP2' or 'RP2350'.
-   **Metadata**: Read `INFO_UF2.TXT` from the mounted drive to confirm model and version.
-   **Flashing Logic**:
    1. Detect mount point.
    2. Copy `firmware.uf2` using `shutil.copy2`.
    3. Wait for unmount/reboot (detecting the loss of the mount point).
    4. Retry up to 3 times on failure.

---

## 7. NASA Power of Ten Compliance ⚖️

1.  **Simple Control Flow**: No recursion, no `goto`.
2.  **Fixed Loop Bounds**: All `while/for` loops have a timeout counter (e.g., `CLOCK_TIMEOUT`).
3.  **No Dynamic Allocation**: `malloc/free` are forbidden after initialization.
4.  **Function Length**: Maximum 60 lines.
5.  **Assertion Density**: Minimum 2 assertions per function (e.g., `ASSERT(pin < 48)`).
6.  **Data Scope**: Declare at smallest possible scope.
7.  **Check Return Values**: The return value of non-void functions must be checked or cast to `(void)`.
8.  **Limited Preprocessor**: No function-like macros. Use inline functions for speed.
9.  **Pointer Limit**: Maximum one level of dereferencing.
10. **Zero Warnings**: `-Wall -Wextra -Werror` required and enforced in `meson.build`.

---

## 8. Glossary
- **UF2**: USB Flashing Format.
- **XIP**: Execute-In-Place from flash mapping.
- **QMI**: Quad SPI Flash interface block.
- **SIO**: Single-cycle IO block (FIFO, Spinlocks, Inter-core).
- **P10**: NASA Power-of-Ten style safety coding rules.
