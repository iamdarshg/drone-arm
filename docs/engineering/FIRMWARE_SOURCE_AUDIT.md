# Issue #22 firmware source and build audit

## Result

The source tree at baseline `1b602e8f7984b3ed6103fe615422991f50b3dfd1` does
not contain the firmware inputs listed by `meson.build`: 36 source files and 13
include directories are absent. The ESC output is described in the build file,
but its target is commented out. A main-board or ESC image cannot be built from
this checkout.

There is no ARM compiler on the initial system PATH, and the environment has no
root access or `sudo`, so no system package installation was attempted. I
staged Debian snapshot-pinned GCC 14.2.1, binutils 2.44, and Newlib packages
under `/tmp`. Cross Meson setup then detected the compiler and linker and failed
at the next blocker: missing `include/`. Package versions, hashes, and captured
setup output are in `engineering_acceptance/evidence/`.

## Reproduce the source audit

From the repository root:

```sh
python3 tools/firmware_source_audit.py \
  --repo . --revision HEAD --expected-revision HEAD \
  --output /tmp/firmware-source-audit.json
```

The audit command exits `1` because required source and include paths are
missing. It resolves the requested revision to a full Git commit, records
Meson source and include paths against that commit's tree, hashes tracked
source and build inputs, parses the Rev-B pin map, and scans available history
for candidate source manifests. Its literal-path scan is an early provenance
check; Meson's setup is the build-graph check.

## Reproduce the staged cross setup without root

The following commands use the signed Debian snapshot URL recorded in the
machine-readable toolchain evidence. They fetch exact package versions,
verify their SHA-256 hashes, and extract them under `/tmp`; they do not install
system packages. They require Debian's `debian-archive-keyring` package to
provide the signing key already referenced by the system's APT config.

The four downloaded package URLs and verified hashes are:

| Package | Version | Snapshot URL | SHA-256 |
|---|---|---|---|
| `gcc-arm-none-eabi` | `15:14.2.rel1-1` | `https://snapshot.debian.org/archive/debian/20260925T181338Z/pool/main/g/gcc-arm-none-eabi/gcc-arm-none-eabi_14.2.rel1-1_amd64.deb` | `99252fdda02ad134e8c3c344d3c843f7aa5c0d006e92a5b3d53daf4bd935b5d3` |
| `binutils-arm-none-eabi` | `2.44-3+23+b1` | `https://snapshot.debian.org/archive/debian/20260925T181338Z/pool/main/b/binutils-arm-none-eabi/binutils-arm-none-eabi_2.44-3%2b23%2bb1_amd64.deb` | `c9c4944e54de852536b4f6a238a682c38a6243cdc2843dd1c4f587e046c0615b` |
| `libnewlib-arm-none-eabi` | `4.5.0.20241231-1` | `https://snapshot.debian.org/archive/debian/20260925T181338Z/pool/main/n/newlib/libnewlib-arm-none-eabi_4.5.0.20241231-1_all.deb` | `b444760d62896f03db89d6db94936929f300b72445763557fa94da32b7732f51` |
| `libnewlib-dev` | `4.5.0.20241231-1` | `https://snapshot.debian.org/archive/debian/20260925T181338Z/pool/main/n/newlib/libnewlib-dev_4.5.0.20241231-1_all.deb` | `feb2ed49a464b0346e42c3506088a41025273bff5f8e6bc5777afeac3704b3c1` |

```bash
set -eu
root=/tmp/issue22-toolchain
mkdir -p "$root/lists/partial" "$root/cache/archives/partial" "$root/debs"
cat > "$root/snapshot.list" <<'LIST'
deb [signed-by=/usr/share/keyrings/debian-archive-keyring.gpg check-valid-until=no] https://snapshot.debian.org/archive/debian/20260925T181338Z/ trixie main
LIST
apt_opts=(
  -o "Dir::Etc::sourcelist=$root/snapshot.list"
  -o Dir::Etc::sourceparts=-
  -o "Dir::State::lists=$root/lists"
  -o "Dir::Cache=$root/cache"
  -o APT::Get::List-Cleanup=0
)
apt-get "${apt_opts[@]}" update
(
  cd "$root/debs"
  apt-get "${apt_opts[@]}" download \
    gcc-arm-none-eabi=15:14.2.rel1-1 \
    binutils-arm-none-eabi=2.44-3+23+b1 \
    libnewlib-arm-none-eabi=4.5.0.20241231-1 \
    libnewlib-dev=4.5.0.20241231-1
  sha256sum -c <<'HASHES'
c9c4944e54de852536b4f6a238a682c38a6243cdc2843dd1c4f587e046c0615b  binutils-arm-none-eabi_2.44-3+23+b1_amd64.deb
99252fdda02ad134e8c3c344d3c843f7aa5c0d006e92a5b3d53daf4bd935b5d3  gcc-arm-none-eabi_15%3a14.2.rel1-1_amd64.deb
b444760d62896f03db89d6db94936929f300b72445763557fa94da32b7732f51  libnewlib-arm-none-eabi_4.5.0.20241231-1_all.deb
feb2ed49a464b0346e42c3506088a41025273bff5f8e6bc5777afeac3704b3c1  libnewlib-dev_4.5.0.20241231-1_all.deb
HASHES
)
mkdir -p "$root/usr"
for deb in "$root"/debs/*.deb; do dpkg-deb -x "$deb" "$root/usr"; done
mkdir -p "$root/wrappers"
cat > "$root/wrappers/arm-none-eabi-gcc" <<'WRAPPER'
#!/bin/sh
exec /tmp/issue22-toolchain/usr/usr/bin/arm-none-eabi-gcc "$@" \
  -B/tmp/issue22-toolchain/usr/usr/lib/arm-none-eabi/newlib/thumb/v8-m.main+fp/hard/ \
  -L/tmp/issue22-toolchain/usr/usr/lib/arm-none-eabi/newlib/thumb/v8-m.main+fp/hard/ \
  --specs=nosys.specs
WRAPPER
chmod +x "$root/wrappers/arm-none-eabi-gcc"
PATH="$root/wrappers:$root/usr/usr/bin:$PATH" \
  meson setup /tmp/issue22-cross-build . --cross-file arm-none-eabi.ini
```

The setup detects GCC 14.2.1 and ld.bfd 2.44, then exits at
`meson.build:33:15` because `include/` does not exist. The result and full
package hashes are captured in
`engineering_acceptance/evidence/firmware_toolchain_probe_1b602e8.json` and
`engineering_acceptance/evidence/meson_cross_setup_with_local_toolchain_1b602e8.log`.
The compiler probe and setup are diagnostic only; they do not produce or
validate firmware.

## Source-history findings

Commit `d28de167fb4f325859349158106b8b82eb51a6ac` planned to remove `src/` and
rebuild from scratch. Commit `fc82f7200c270dfa63f20a2f7ad9c004f905de8e`
performed that deletion, removing 100 tracked files and 10,518 lines. The
history records the intent to start fresh, but does not identify a validated
replacement implementation.

The separate candidate `6ca24e23dd40fcbed4dab338969a6b0513e212d6` on
`origin/sdk-recreation` contains 38 `src/` files. Its Meson graph emits one
generic `firmware.elf`/`.bin`/`.uf2` in a cross build. Its `main()` only starts
the board and scheduler; `esc_controller.c` says it was superseded by
`pwm_output.c`, a generic RC-PWM driver that uses a busy-wait. It contains no
separate cell image, no STM32G431 local FOC firmware, and no implementation of
the six isolated UART command links.

The [Rev-B architecture](../hardware/REV_B_ARCHITECTURE.md) specifies six
STM32G431 motor controllers running local FOC and an RP2354B supervisor. The
board pin assignments are recorded in
[`mcu_pinmap.json`](../../hardware/main/rev_b/mcu_pinmap.json). The historical
generic RP2350 scheduler/PWM candidate is not tied to those interfaces or
assignments, so its presence in Git history does not establish that it can
replace the missing firmware.

## Validation limits

This audit reports provenance and buildability. It does not establish target
boot, correct PWM, safe commutation, or physical safety.
