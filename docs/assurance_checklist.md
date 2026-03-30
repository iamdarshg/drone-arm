# Assurance Checklist – Drone-Arm Firmware

## Prioritised Immediate Actions (Top 10)

1. **Enforce CI on every PR** – block merges until `.github/workflows/ci.yml` passes (build + tests + static analysis).
2. **Zero-tolerance static analysis** – run `cppcheck` and `clang-tidy` with exit-code enforcement; fix all current warnings.
3. **Require linked requirement IDs in every PR** – `SPEC.md` section numbers (e.g. `SPEC §7.2`) must appear in PR descriptions.
4. **Enable coverage gate** – generate `lcov` HTML report; fail CI if line coverage drops below 70 %.
5. **Formalise fault and watchdog handling** – document and test the software watchdog path and hard-fault handler in `startup.S`.
6. **Establish WCET budget comments** – annotate all ISR-context and scheduler dispatch functions with measured worst-case execution estimates.
7. **Lock toolchain versions** – pin `arm-none-eabi-gcc`, `clang-tidy`, `cppcheck`, `meson`, and `ninja` versions in `ci/Dockerfile`.
8. **Memory safety audit** – run `valgrind` / AddressSanitizer on all host-side tests; add to CI matrix.
9. **Secure boot integrity** – document and test the boot2 CRC32 path; gate firmware flashing behind checksum validation.
10. **Reproducible builds** – verify that two clean builds from the same commit produce byte-identical `firmware.uf2`; add artifact hash check to CI.

---

## Requirements Traceability

- [ ] Every source module has a `SPEC §` reference in its header comment.
- [ ] All public API functions are covered by at least one test.
- [ ] `sensor_requirements.md` items are cross-linked to `state_vector.c` test assertions.
- [ ] Requirements changes trigger downstream test review (tracked in PR body).
- [ ] SPEC.md version is recorded in firmware banner string.

## Testing

- [ ] Host-side unit tests pass on `main` and every feature branch (`meson test`).
- [ ] Test coverage ≥ 70 % line coverage on `src/` (measured by `gcov`/`lcov`).
- [ ] All test executables exit non-zero on failure (verified by CI matrix).
- [ ] No test executable skips assertions in release mode (`-DNDEBUG` must not disable TEST_ASSERT).
- [ ] Python integration tests (`test_build_artifacts.py`, `test_pad_checksum.py`) pass on fresh checkout.
- [ ] Regression suite is run on both x86-64 host and ARM cross-compiled output where applicable.

## Static Analysis

- [ ] `cppcheck --error-exitcode=1 --enable=all` passes with zero findings.
- [ ] `clang-tidy` passes with project `.clang-tidy` configuration and zero warnings.
- [ ] No `-Wunused-*`, `-Wshadow`, or `-Wformat-*` warnings at `-Wall -Wextra -Werror`.
- [ ] All pointer arithmetic guarded by range checks or ASSERT macros.
- [ ] No `malloc`/`free` in interrupt context or scheduler hot path (NASA Power of Ten rule 3).

## WCET (Worst-Case Execution Time)

- [ ] Scheduler dispatch (`scheduler_tick`) WCET annotated and measured at target clock.
- [ ] Each ISR has a cycle-count comment upper-bound verified by `arm-none-eabi-nm` + manual review.
- [ ] PWM output update path WCET ≤ 10 µs at 150 MHz.
- [ ] State-vector write path WCET ≤ 5 µs.
- [ ] DMA interrupt handler WCET documented and within budget.

## Memory Safety

- [ ] No dynamic allocation in safety-critical paths (`malloc` lint check in CI).
- [ ] Stack size per task is documented in `scheduler.h` and verified at link time.
- [ ] All ring-buffer and FIFO boundaries checked with ASSERT.
- [ ] AddressSanitizer clean on all host-side tests.
- [ ] No integer overflow in divisor calculations (verified by `test_driver_math`).

## Fault Handling

- [ ] Hard-fault handler in `startup.S` captures stack frame and outputs diagnostic UART message.
- [ ] Software watchdog bites within configurable timeout and resets processor.
- [ ] `ASSERT` macro in `src/kernel/assert.c` triggers fault log before halt.
- [ ] Fault handling code paths exercised by dedicated test or simulation.
- [ ] Recovery strategy documented for each fault class (SPEC §10 or equivalent).

## Boot and Update Security

- [ ] `boot2.S` CRC32 validated at startup; firmware halts on mismatch.
- [ ] UF2 family ID checked before flashing (`UF2_RP2350_ARM_FAMILY_ID`).
- [ ] `device_autodetect.py` verifies UF2 magic bytes before copy.
- [ ] OTA / USB update path requires valid CRC and correct family ID.
- [ ] Debug interface (SWD) disabled in production build (linker flag or register setting documented).

## PCB / EMC

- [ ] Motor drive PWM frequency and duty cycle limits documented in `pwm_output.h`.
- [ ] EMC-sensitive GPIO slew rates and drive strengths set in `board_config.h`.
- [ ] Decoupling capacitor placements noted in `hardware/` BOM or schematic.
- [ ] Ground-plane continuity verified in layout review checklist (see `hardware/`).

## Observability

- [ ] UART banner logs firmware version, build hash, and clock frequencies on boot.
- [ ] Scheduler task run-time statistics accessible via debug UART command.
- [ ] Sensor state-vector values loggable via debug command without halting system.
- [ ] Wokwi simulation `tests/wokwi/` updated to reflect latest pin assignments.

## CI – Reproducible Builds

- [ ] `ci/Dockerfile` pins all tool versions; image digest tracked in CI workflow.
- [ ] `ninja -C builddir` produces identical output for the same commit (bit-for-bit).
- [ ] CI uploads `firmware.uf2` + SHA-256 digest as a workflow artifact.
- [ ] CI workflow exits non-zero on build failure, test failure, or static-analysis findings.
- [ ] Branch protection rules require CI green before merge.

## Documentation

- [ ] `README.md` covers configure / build / test / flash workflow.
- [ ] `SPEC.md` is up to date with implemented scheduler, sensor, and driver APIs.
- [ ] `docs/assurance_checklist.md` (this file) reviewed and updated each sprint.
- [ ] `docs/90_day_roadmap.md` milestones tracked weekly.
- [ ] API doxygen-style comments present on all public functions in `src/`.

## Security

- [ ] No hardcoded credentials or keys in source tree.
- [ ] Dependency supply-chain: all third-party sources pinned to a known-good commit or version.
- [ ] `cppcheck` security checks enabled (`--enable=unusedFunction,warning,portability`).
- [ ] clang-tidy `bugprone-*` and `cert-*` checks enabled.
- [ ] Threat model documented for USB/UF2 update path.

---

## Verification Metrics

| Metric | Current | Target |
|---|---|---|
| Unit test count | 8 | ≥ 15 |
| Line coverage (host tests) | ~40 % | ≥ 70 % |
| Static analysis warnings | unknown | 0 |
| WCET annotations | 0 | all ISRs |
| Docs coverage (public API) | partial | 100 % |
| CI pass rate (30 days) | N/A | ≥ 95 % |

---

## 90-Day Roadmap Summary

See [`docs/90_day_roadmap.md`](90_day_roadmap.md) for the full milestone plan.

**Week 0–2:** CI foundation, static analysis baseline, coverage gate  
**Week 2–6:** WCET annotations, memory safety, fault handler tests  
**Week 6–12:** PCB review, observability, security hardening, documentation freeze  
