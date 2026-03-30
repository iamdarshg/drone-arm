# Drone-Arm Firmware

Low-level RP2350 firmware for multicore scheduling, sensor ingestion, and motor-control outputs, built with Meson + Ninja and validated with host-side tests.

## Build System Overview

This repository uses:
- **Meson** for configuration and build graph generation
- **Ninja** as the backend executor
- **arm-none-eabi** toolchain for cross-compiled firmware artifacts
- **Python 3** for host-side validation tools and artifact checks

The main build definition is in `meson.build`.

## Prerequisites

- `python3`
- `meson`
- `ninja`
- `arm-none-eabi-gcc` / `arm-none-eabi-objcopy` (for cross builds and firmware artifacts)

## Configure

```bash
meson setup builddir
```

Reconfigure an existing directory:

```bash
meson setup builddir --reconfigure
```

## Build

### Host tests and native binaries

```bash
ninja -C builddir
```

This builds native test executables such as:
- `test_uf2`
- `test_scheduler`
- `test_state_vector`
- `test_clock_safety`
- `test_pwm_output`
- `test_driver_math`

### Cross firmware artifacts

When Meson is configured as a cross build (`meson.is_cross_build()`), the build also emits:
- `boot2.bin`
- `boot2_padded.S`
- `firmware.elf`
- `firmware.bin`
- `firmware.uf2`

## Test Workflow

Run all tests:

```bash
meson test -C builddir
```

Run specific tests:

```bash
meson test -C builddir scheduler_test state_vector_test driver_math_test
```

Current test groups include:
- `uf2_struct_test` (`tests/test_uf2.c`)
- `uf2_artifact_test` (`tests/test_build_artifacts.py`)
- `pad_checksum_test` (`tests/test_pad_checksum.py`)
- `scheduler_test` (`tests/test_scheduler.c`)
- `state_vector_test` (`tests/test_state_vector.c`)
- `clock_safety_test` (`tests/test_clock_safety.c`)
- `pwm_output_test` (`tests/test_pwm_output.c`)
- `driver_math_test` (`tests/test_driver_math.c`)

Coverage for recently added functionality:
- `driver_math_test`: UART/SPI/I2C math plus DMA DREQ and I2C DMA command-format checks.
- `scheduler_test`: task memory-region ownership/shared-access policy checks.
- `state_vector_test`: async IMU/GPS registration/readiness and shared state-vector surface checks.

## Device Auto-Detection and Flashing

Tool: `tools/device_autodetect.py`

What it does:
1. Scans mounted block devices (`lsblk`) for `RP2350` / `RPI-RP2` labels.
2. Reads `INFO_UF2.TXT` when present.
3. Copies the supplied UF2 to the mounted BOOTSEL drive.
4. Waits for unmount/reboot and retries on failure.

Usage:

```bash
python3 tools/device_autodetect.py builddir/firmware.uf2 --retry 3
```

If no device is found, place the board into BOOTSEL mode and run the command again.
