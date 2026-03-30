# 90-Day Assurance Roadmap – Drone-Arm Firmware

## Week 0–2: CI Foundation

- [ ] Merge `ci/Dockerfile` and `.github/workflows/ci.yml` to sdk-recreation.
- [ ] Enable branch protection: require CI passing before merge.
- [ ] Run `cppcheck` and `clang-tidy` on full `src/`; triage all findings into a tracking issue.
- [ ] Establish baseline line-coverage report (`lcov`); record percentage in a pinned issue.
- [ ] Ensure all existing tests pass in CI (`meson test -C builddir`).
- [ ] Add `.github/PULL_REQUEST_TEMPLATE.md`; update contribution guidelines.
- [ ] Verify reproducible build: two CI runs on the same commit produce identical `firmware.uf2`.

## Week 2–6: Quality and Safety Hardening

- [ ] Raise test count from current baseline to ≥ 15 tests across `src/` modules.
- [ ] Annotate all ISR and scheduler hot-path functions with WCET budget comments.
- [ ] Enable AddressSanitizer (`-fsanitize=address`) in host-side test builds in CI.
- [ ] Document stack size per scheduler task in `scheduler.h`; add link-time assertion.
- [ ] Write tests for hard-fault handler and software watchdog paths.
- [ ] Enforce `--error-exitcode=1` on `cppcheck`; fix all remaining findings.
- [ ] Add `clang-tidy` config (`.clang-tidy`) enabling `bugprone-*` and `cert-*` checks.
- [ ] Close all P0 static-analysis findings from Week 0–2 triage.
- [ ] Add `malloc`-free lint check (grep or cppcheck rule) to CI.
- [ ] Expand `test_driver_math` to cover edge cases (min/max clock, baud error).

## Week 6–12: Security, Observability, and Documentation Freeze

- [ ] Complete PCB/EMC review checklist in `hardware/`; record findings.
- [ ] Document all EMC-sensitive GPIO settings in `board_config.h`.
- [ ] Add observability debug-UART command to dump scheduler stats and state-vector.
- [ ] Verify `boot2` CRC32 check is exercised by a host-side or simulation test.
- [ ] Write threat model document for USB/UF2 update path.
- [ ] Achieve ≥ 70 % line coverage gate enforced in CI (fail if coverage drops).
- [ ] Complete doxygen-style API comments on all public functions in `src/`.
- [ ] Update `SPEC.md` to reflect final implemented API surface.
- [ ] Freeze `docs/assurance_checklist.md`; sign off all checklist items or defer with rationale.
- [ ] Conduct final reproducible-build audit; publish SHA-256 digest of release `firmware.uf2`.
- [ ] Tag `v1.0.0-rc1` and run full CI matrix; document results.

---

## Weekly Metrics to Track

| Week | Test count | Line coverage | Open static-analysis findings | WCET-annotated functions |
|------|-----------|--------------|-------------------------------|--------------------------|
| 0    | 8         | TBD          | TBD                           | 0                        |
| 2    | ≥ 10      | baseline     | all triaged                   | 0                        |
| 4    | ≥ 12      | ≥ 50 %       | P0 closed                     | ISRs done                |
| 6    | ≥ 15      | ≥ 60 %       | P1 closed                     | all hot paths            |
| 8    | ≥ 18      | ≥ 65 %       | 0                             | all public APIs          |
| 10   | ≥ 20      | ≥ 70 %       | 0                             | complete                 |
| 12   | ≥ 20      | ≥ 70 %       | 0                             | complete                 |
