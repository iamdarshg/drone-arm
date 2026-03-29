# Drone-Arm Firmware Technical Specification

Version: 0.2.0  
Date: 2026-03-29  
Target: RP2350B (dual Cortex-M33)  
Toolchain: arm-none-eabi-gcc (cross), host GCC/Python for tests

## Status Legend
- ✅ Complete
- 🔧 Partial / needs improvement
- ❌ Not yet implemented
- 📝 Spec defined, not implemented

## 1. Build System ✅

### 1.1 Toolchain and build architecture
This firmware is built with Meson + Ninja and targets RP2350B directly in a clean-room approach, with no SDK code copied into the tree.

Cross compile profile:
- CPU: Cortex-M33
- ISA: Thumb-2
- FPU ABI: hard
- FPU: FPv5-D16

Core compile flags and intent:
- `-mcpu=cortex-m33`: emit M33 instructions and scheduling assumptions.
- `-mthumb`: generate Thumb-2 code (required for Cortex-M boot conventions).
- `-mfloat-abi=hard`: pass FP values in FPU registers.
- `-mfpu=fpv5-d16`: match M33 FPU unit.
- `-ffunction-sections -fdata-sections`: isolate symbols to maximize dead-code stripping.
- `-Wl,--gc-sections`: link-time elimination of unused sections.
- `--specs=nosys.specs --specs=nano.specs`: embedded libc profile with no OS syscalls.

Build mode behavior:
- Release (default): `-O2 -DNDEBUG`
- Debug: `-g -O0`

### 1.2 Output artifacts
Primary artifacts:
- `firmware.elf`: linked executable image with vectors and all sections.
- `firmware.uf2`: flash-ready UF2 file.
- `boot2.elf`, `boot2.bin`, `boot2_padded.S`: boot2 pipeline for ROM consumption.

Test artifacts:
- `test_runner` (host-native C tests)
- Python artifact checks (UF2 structure and addressing checks)

### 1.3 UF2 format requirements
UF2 block format is 512 bytes per block:
- 32-byte header
- up to 476-byte payload field in this project’s C representation
- 4-byte end magic

Mandatory magic values:
- start0: `0x0A324655`
- start1: `0x9E5D5157`
- end: `0x0AB16F30`

Family ID for RP2350 ARM firmware:
- `0xE48BFF57`

### 1.4 Boot2 checksum path
`tools/pad_checksum` pads boot2 binary to 252 bytes and appends 4-byte checksum word, then emits assembly (`boot2_padded.S`) linked into firmware.

### 1.5 Build commands
Typical flow:
```bash
meson setup builddir --cross-file arm-none-eabi.ini
ninja -C builddir
```
Tests:
```bash
meson test -C builddir
```

## 2. Memory Map ✅

Core memory regions used by this design:
- ROM: `0x00000000`
- XIP flash window: `0x10000000`
- XIP SRAM/cache alias: `0x13FFC000`
- Main SRAM: `0x20000000`
- Scratch X: `0x20080000`
- Scratch Y: `0x20081000`

Primary peripheral spaces:
- APB region starts at `0x40000000`
- DMA at `0x50000000`
- PIO blocks at `0x50200000`, `0x50300000`, `0x50400000`
- SIO at `0xD0000000`

Linker section model:
- `.vectors` at flash base for reset and exception entry.
- `.text/.rodata` in flash.
- `.data` copied flash->RAM during reset.
- `.bss` zeroed during reset.
- `.ramfunc` copied to RAM for optional low-latency paths.

## 3. Startup Sequence ✅

Reset path summary:
1. ROM boots and validates stage-2 boot code.
2. Boot2 applies minimal QMI/XIP config then returns.
3. Vector table at flash base is used to load MSP and reset handler.
4. `reset_handler` initializes RAM sections.
5. `main()` performs board init, multicore init, scheduler init.

Runtime launch model:
- Core0 launches core1 entry.
- Both cores run cooperative scheduler loops.

## 4. Boot Stage 2 🔧

Boot2 responsibilities in this project:
- Keep size bounded for ROM boot expectations.
- Set up initial QMI read behavior for flash fetch.
- Return control to ROM/startup path cleanly.

Current status:
- pipeline and placement are defined
- hardware timing validation across flash variants is pending

## 5. Clock Tree ✅

Configured baseline:
- External crystal path brought up.
- PLL_SYS configured to target 150 MHz system clock.
- `clk_ref`, `clk_sys`, and `clk_peri` configured for deterministic peripheral timing.

Known improvements:
- dynamic scaling and measured lock-time instrumentation are future work.

## 6. GPIO ✅

GPIO control combines:
- IO bank function select
- pad settings (pull, drive, schmitt, input enable)
- SIO data and direction registers for software-driven pins

Policy:
- explicit pin-range checks (`pin < 48`)
- explicit direction and pull configuration per init call

## 7. I2C ✅

Controller mode:
- DW APB I2C style programming model
- master operation with programmable SCL timing

Current implementation focus:
- polled read/write transactions
- deterministic, bounded transaction loops

Future enhancements:
- abort source decoding and retry strategy
- bus clear and recovery flow

## 8. SPI ✅

Controller mode:
- master
- 8-bit Motorola framing

Transfer model:
- polling against FIFO/status flags
- explicit end-of-transfer busy wait

Future enhancements:
- DMA-backed full-duplex bursts
- interrupt-driven low-latency queueing

## 9. UART ✅

Baseline mode:
- PL011-style programming model
- 8N1 framing
- FIFO enabled

Current APIs target:
- blocking `putc/getc`
- minimal dependency footprint

Future work:
- buffered/non-blocking transport
- IRQ-driven RX with ring buffer

## 10. DMA 🔧

Current scope:
- channel-level register programming and status checks
- busy polling for completion

Pending:
- chaining
- ring mode
- peripheral pacing maps as configurable policy

## 11. Multicore ✅

Synchronization hardware:
- SIO FIFO for inter-core messaging
- hardware spinlocks for mutual exclusion

Launch model:
- core0 prepares and triggers core1 startup handshake
- both cores join scheduler runtime

## 12. Scheduler ✅

Scheduler type:
- cooperative, bounded task table
- static task storage, no dynamic allocation

Core features:
- create/kill/task query
- sleep/yield/wait primitives
- per-core queues
- simple stats collection

## 13. Compilation Pipeline ✅

Pipeline:
1. C/ASM compile to objects
2. link to ELF with linker script
3. optional binary extraction
4. UF2 conversion

Boot2 pipeline is integrated as pre-link generated assembly.

## 14. Board Configuration ✅

Board profile defines pin mapping for:
- SPI0 and SPI1
- I2C0 and I2C1
- UART0
- ESC PWM outputs
- LED/status pin

Board init applies peripheral clocks, watchdog, and pin mux setup.

## 15. Device Auto-Detection ❌

Not yet implemented.

Planned direction:
- detect BOOTSEL USB mass-storage appearance
- verify INFO_UF2 metadata
- automated retry flashing workflow

## 16. PIO ❌

Not yet implemented.

Planned direction:
- state machine setup APIs
- instruction encoder helpers
- DMA-assisted transfer support

## 17. ESC Controller ❌

Not yet implemented in the clean-room rewrite.

Planned direction:
- commutation and sensing interfaces
- deterministic control loops
- scheduler integration per motor channel

## 18. Testing Strategy ✅

Current tests:
- scheduler host tests
- UF2 structure tests (`tests/test_uf2.c`)
- UF2 artifact scanner (`tests/test_build_artifacts.py`)

Goals:
- verify generated image metadata
- enforce target-address and magic consistency
- keep host tests independent from hardware presence

## 19. Known Limitations & Follow-Up 🔧

1. Hardware validation not yet completed for all drivers.
2. Interrupt-driven paths largely pending.
3. UF2 artifact test is format-level, not execution-level.
4. Boot2 timing and flash-variant tuning pending.

## 20. Glossary ✅

- UF2: USB Flashing Format with fixed-size blocks.
- XIP: Execute-In-Place from flash mapping.
- QMI: Flash interface block used for memory reads.
- SIO: Single-cycle IO block, includes FIFO/spinlocks.
- P10: NASA Power-of-Ten style safety coding rules.
