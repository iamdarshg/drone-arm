# Issue #22 firmware source and build audit

## Result

The current source snapshot does not contain the firmware source files named by
`meson.build`. The main-board target names 37 absent C/assembly/linker paths and
12 absent include directories. The ESC output is described in the build file,
but its target is commented out. A main-board or ESC image cannot be built from
this checkout.

This PR records a reproducible provenance check and the blocked build result.
It does not restore firmware or establish compatibility with the Rev-B board.
The cross build is independently blocked by the missing `arm-none-eabi-gcc`
toolchain. The available historical source candidate is not a valid replacement
for the present Rev-B architecture; see `/workspace/firmware-history.md` for
the full source-history analysis and limitations.

## Reproduction

From the repository root, run:

```sh
python3 tools/firmware_source_audit.py \
  --repo . --revision HEAD --expected-revision HEAD \
  --output /tmp/firmware-source-audit.json
meson setup /tmp/drone-arm-cross-build . --cross-file arm-none-eabi.ini
```

The audit command exits `1` because required source and include paths are
missing. It resolves the requested revision to a full Git commit, records
Meson source and include paths against that commit's tree, hashes available
source and build inputs, parses the Rev-B pin map, and scans available history
for candidate source manifests. Its literal-path scan is an early provenance
check; Meson's setup is the build-graph check.

On baseline `1b602e8f7984b3ed6103fe615422991f50b3dfd1`, native Meson setup exits
`1` at `meson.build:33` because `include/` is absent. Cross Meson setup exits
`1` at compiler discovery because `arm-none-eabi-gcc` is not installed. The
captured outputs and machine-readable audit are in
`engineering_acceptance/evidence/`. The audit's expected revision is pinned to
that baseline so the receipt fails if it is silently generated from another
source revision.

## Provenance conclusion

The repository history identifies `fc82f7200c270dfa63f20a2f7ad9c004f905de8e`
as the commit that removed the prior `src/` and `include/` trees. The separate
historical candidate `6ca24e23dd40fcbed4dab338969a6b0513e212d6` is a generic
RP2350 experimental scheduler/sensor skeleton. It has no distinct ESC image,
does not implement the documented STM32G431 six-cell ESC architecture, and is
not tied to the current Rev-B pin map. Its presence in Git history is not
evidence that it can safely replace the missing firmware.

The audit reports source provenance and buildability only. Compilation would
not prove target boot, correct PWM, safe commutation, or physical safety.
