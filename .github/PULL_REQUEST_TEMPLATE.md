## Summary

<!-- One-sentence description of what this PR does and why. -->

## Linked Requirements

<!-- List every SPEC.md section or sensor_requirements.md item touched.
     Example: SPEC §7.2 (scheduler dispatch), SPEC §4.1 (UART init) -->

- SPEC §

## Changed Modules

<!-- List each source file changed and a brief reason. -->

| File | Change |
|------|--------|
| `src/` | |

## Tests Added / Updated

<!-- Describe new or modified tests and what they verify. -->

- [ ] New test(s) added in `tests/`
- [ ] Existing tests updated to cover changed behaviour
- Coverage delta: (e.g. +2 % line coverage on `src/kernel/`)

## CI Status

- [ ] CI workflow passes (build, tests, cppcheck, clang-tidy)
- [ ] No new static-analysis warnings introduced
- [ ] Coverage does not regress

## Documentation

- [ ] `SPEC.md` updated if API surface changed
- [ ] `README.md` updated if build or test workflow changed
- [ ] Inline comments / doxygen updated for changed functions

## Assurance Checklist Reminder

- [ ] All assertions and ASSERT macros are preserved (not removed for passing tests)
- [ ] No dynamic allocation added to interrupt or scheduler hot paths
- [ ] WCET budget comment added / preserved for any ISR or dispatch function touched
- [ ] Fault handling paths remain intact and tested

## Requested Reviewers

<!-- Tag at least one reviewer who understands the changed subsystem. -->

@
