# Project TODO & Debugging Plan — rp2350 build / UF2 issues

Overview
--------
You reported that the firmware doesn't run on your `rp2350`. The build output shows linker errors referencing `GPIO_FAST_OP` being undefined, and you suspect either UF2 conversion or compilation/target mismatches. This TODO collects the investigation steps, prioritized fixes, and a concrete plan to make the firmware bare-metal (no direct SDK usage). Because you requested no direct SDK use, any small assembly wrappers or coprocessor helper functions we need must be copied into our own headers and kept inline/static to avoid external dependencies.

High-level goals
- Get a reproducible build for the rp2350 that links and produces a working `.elf` / `.bin` / `.uf2`.
- Replace or fix `GPIO_FAST_OP` so it compiles and links correctly for rp2350.
- Provide a minimal, copy-local set of assembler wrappers (copied from SDK style) in our own header(s) so we are bare-metal.
- Establish a reproducible diagnostic and UF2 creation workflow and document it here.

Current immediate failure (from build logs)
- Linker fails with undefined references to `GPIO_FAST_OP` coming from `src/main_board/low_level/gpio.c`.
- The file `src/main_board/low_level/gpio.c` *contains* a `void GPIO_FAST_OP(int op, int mask) { ... }` definition, but the linker still reports it as undefined.
- Possible causes: assembler / inline-asm incompatibility with target flags, function optimized away or made local, target mismatch (rp2040 vs rp2350), or code emitted differently than expected.

Top-level TODO (prioritized)
1. Diagnostics: confirm what object files actually contain for `gpio.c`.
   - Commands to run locally (repo root `D:\CodeProjects\drone-arm`):
     - `ninja -C build -v`  // verbose rebuild
     - `arm-none-eabi-nm build/src_main_board_low_level_gpio.c.o | grep GPIO_FAST_OP || true`
     - `arm-none-eabi-objdump -d build/src_main_board_low_level_gpio.c.o | sed -n '1,240p'`
     - `arm-none-eabi-readelf -s build/src_main_board_low_level_gpio.c.o | grep -i gpio || true`
     - Recompile `gpio.c` to assembly for review:
       - `arm-none-eabi-gcc -c -save-temps -Og -S -mcpu=cortex-m33 -mthumb -o /dev/null src/main_board/low_level/gpio.c`
   - Expected checks:
     - `nm` should show `GPIO_FAST_OP` as a defined symbol (type `T` or `t`).
     - `objdump` should show generated assembly for the function.
     - If `GPIO_FAST_OP` is not present, deduce why it was removed or not emitted.

2. Replace `GPIO_FAST_OP` with local, static inline coprocessor wrappers (copy of SDK wrappers)
   - Rationale:
     - Inline wrappers avoid external symbol emission.
     - We can't depend on SDK headers: copy minimal assembly wrappers into our own header(s).
     - Use `static inline` + `__asm__ volatile` to ensure proper emission and inlining.
   - Files to add (suggested):
     - `src/main_board/low_level/gpio_coproc_local.h` (exports `static inline` functions)
     - Update `src/main_board/low_level/gpio.c` to `#include "gpio_coproc_local.h"` and replace `GPIO_FAST_OP()` calls with small internal wrapper calls (or rename the function to `gpio_fast_op()` and implement as `static inline`).
   - Example mapping (we must adapt op -> wrapper):
     - op 0 -> `gpioc_lo_out_set(x)` or `gpioc_lo_out_put(x)` depending on semantics
     - op 1 -> `gpioc_lo_out_clr(x)`
     - op 2 -> `gpioc_lo_out_xor(x)`
     - op 4 -> `gpioc_lo_oe_set(x)` (output enable set)
     - op 5 -> `gpioc_lo_oe_clr(x)` (output enable clear)
     - For high bank (pins >= 32) use hi variants or direct SIO registers.

3. Rebuild and verify object/ELF
   - Rebuild: `ninja -C build`
   - Confirm `nm`/`readelf` shows no undefined references.
   - If undefined symbol persists, capture full link command and full linker error output.

4. UF2 conversion / flashing
   - Once ELF is valid, convert to UF2 using a local copy of conversion script or adhoc tool:
     - Preferred: repo-provided `elf2uf2` or `tools/elf2uf2/elf2uf2.py` (if present).
     - If none exists, craft a minimal UF2 creation flow (but prefer the tested tool).
   - Make sure the UF2 `target-family` GUID and metadata matches rp2350 bootloader expectation. Using rp2040 GUID will fail.
   - Flash with the standard drag/drop method or board-specific loader.

5. Document the final working flow & integrate into Meson/Ninja
   - Add the local header into the build include path.
   - Ensure `meson.build` references the new header (no linking to SDK).
   - Commit changes and update `TODO.md` with final steps.

Detailed debugging plan for `GPIO_FAST_OP`
- A. Inspect compiled object:
  - Run `arm-none-eabi-nm` on the `gpio.c.o` and check whether `GPIO_FAST_OP` is present and whether its symbol is local (`t`) or global (`T`).
  - If the function was compiled as `static` or inlined away, symbol type may differ — we must ensure it remains accessible from other compilation units if needed. But ideally we make it `static inline` and call local inline wrappers from each C file that needs it to avoid cross-object linkage.
- B. Inspect assembly emitted:
  - `objdump -d` the object file and locate the function body. Confirm correct encoding of `mcr` instruction lines.
- C. If assembly invalid or missing:
  - Replace the current `GPIO_FAST_OP` with a `static inline` set of named wrappers (see next section) and recompile.
- D. If linking still fails:
  - Capture full link command and check object list ordering; verify that `-Wl,--as-needed` or other flags are not stripping things unexpectedly.
  - Check for mismatched calling convention (unlikely in C) or symbol name mangling (shouldn’t happen with plain C).
- E. If coprocessor ops are invalid for cortex-m33:
  - The `mcr` instructions are RP2040-specific and intended for RP2040's coprocessor usage. If the rp2350 core does not support these exact `mcr` encodings, the assembler will reject them or encode them differently — you may need to implement direct SIO register writes for the rp2350 chip (write to `SIO_BASE + offset`), or use the rp2350-provided fastest mechanism.
  - Confirm the `mcr` instruction semantics for rp2350; if not supported, use memory-mapped SIO register writes.

Concrete code approach (no SDK imports — copy only the minimal wrappers)
- Create `src/main_board/low_level/gpio_coproc_local.h` that defines a small set of `static inline` functions (copy/paste of the SDK wrappers, but self-contained). Example recommended contents (adapted for our repo; ensure the path included by `gpio.c`):

Contents suggestion for `src/main_board/low_level/gpio_coproc_local.h`:
- Provide a small macro / inline-asm helper, and define low/high bank functions as `static inline`.
- Use names prefixed to avoid collision: `local_gpioc_lo_out_set`, etc.
- Keep functions `static inline` so they are not emitted as linkable symbols.

Example (to copy into the new header):
- Use `__asm__ volatile ("mcr p0, #<opc>, %0, c<0>, c<0>" : : "r" (x));` form for low out ops.
- Use `c4` for OE ops as seen in existing SDK wrappers, and `c1` for hi bank variants (see SDK mapping).
- Document these wrappers carefully (source attribution: derived from SDK style; adapted for local use).

(Important note: exact `mcr` operands and `#` immediates must match the SDK wrappers you observed. The header below is an example you should include exactly in the file tree as `src/main_board/low_level/gpio_coproc_local.h`.)

Suggested header skeleton (text form to be saved as the new header):
- `#ifndef MAIN_BOARD_LOW_LEVEL_GPIO_COPROC_LOCAL_H`
- `#define MAIN_BOARD_LOW_LEVEL_GPIO_COPROC_LOCAL_H`
- `static inline void local_gpioc_lo_out_put(uint32_t x) { __asm__ volatile ("mcr p0, #0, %0, c0, c0" : : "r"(x)); }`
- `...` (similar wrappers for `out_xor`, `out_set`, `out_clr`, `hi_*` variants, `lo_oe_*` and `hi_oe_*`) 
- `#endif`

(You will add this file; then change `gpio.c` calls from `GPIO_FAST_OP(op, mask)` to a small local inline `gpio_fast_op(op, mask)` that dispatches to the local_gpioc_* functions in `gpio_coproc_local.h`.)

Example `gpio.c` change (conceptual)
- At top: `#include "gpio_coproc_local.h"`
- Replace:
  - `void GPIO_FAST_OP(int op, int mask) { switch(op) { case 0: __asm__ volatile ("mcr ..."); ... } }`
- With:
  - `static inline void local_gpio_fast_op(int op, int mask) { switch (op) { case 0: local_gpioc_lo_out_set(mask); break; ... } }`
  - Replace all callers `GPIO_FAST_OP(...)` with `local_gpio_fast_op(...)` or call the appropriate low/high wrapper inline directly from callsites.

Why this approach avoids the current link error
- `static inline` wrappers are inlined at call sites; no external symbol is emitted and thus nothing to be resolved by the linker.
- Copying only the tiny wrappers keeps the codebase bare-metal and independent of SDK headers while retaining the same inline assembly patterns that were used in the SDK.
- If the `mcr` op is incompatible with the target CPU/assembler, the asm will fail at compile/assemble time; at that stage we'll know the instruction set mismatch and can switch to explicit SIO register writes.

If the coprocessor assembly is not supported for rp2350
- Fallback: implement direct SIO register writes. The repo already includes `addressmap.h` and `structs/sio.h` with offsets; use `REG32_WRITE(SIO_BASE + SIO_GPIO_OUT_SET_OFFSET, mask)` style writes for the low/high banks instead of coproc ops.
- Using memory mapped writes is slightly slower but fully portable across cores that expose the SIO block.

Detailed test & verification steps
1. Implement the header + `gpio.c` changes (local wrappers).
2. Build: `ninja -C build`.
3. If build succeeds, verify `nm`/`readelf` that no undefined `GPIO_FAST_OP` remains:
   - `arm-none-eabi-readelf -s build/firmware_main_board.elf | grep -i GPIO_FAST_OP || true`
4. Run tests (if available) or flash on board:
   - Convert ELF -> UF2 (local tool): `python3 tools/elf2uf2/elf2uf2.py firmware_main_board.elf firmware_main_board.uf2` (adjust path/tool as needed)
   - Drag-drop `.uf2` to the board if it presents as mass storage, or flash via `picotool`/`openocd` as appropriate for rp2350.
5. If board does not boot: collect serial logs, link map, and `dmesg` on host to see mount/boot messages.

Failure escalation
- If assembler rejects `mcr` opcodes:
  - Capture the assembler error and the exact compile args; we will adapt the inline asm to valid instructions or use memory-mapped SIO registers.
- If `GPIO_FAST_OP` still reported as undefined after replacement:
  - Re-check for stale object files, cleaning `build/` and a full rebuild: `ninja -C build -t clean` or remove `build/` and re-run Meson/Ninja configure/build.
- If UF2 fails to flash or boot:
  - Check UF2 `target-family` GUID; create UF2 with correct GUID for rp2350. If unknown, check board documentation, or test with the `.bin` via `picotool load` or `openocd` to confirm ELF is valid before uf2 conversion.

Files to create/update (concrete)
- Create: `src/main_board/low_level/gpio_coproc_local.h` (copy minimal wrappers here)
- Update: `src/main_board/low_level/gpio.c`
  - `#include "gpio_coproc_local.h"`
  - Replace `GPIO_FAST_OP` with `static inline` local wrapper dispatching to `local_gpioc_*` functions
- Update Meson include paths if necessary (to include `src/main_board/low_level`).
- Add unit/build debug target that compiles `gpio.c` to assembly for quick checks.

Acceptance criteria
- Clean build: `ninja -C build` completes with zero link errors.
- `arm-none-eabi-nm` on the object files shows no undefined `GPIO_FAST_OP`.
- Board boots the new firmware and basic GPIO operations work (toggle LED or read input).
- UF2 produced with correct metadata flashes successfully and board runs firmware.

Next immediate action (I will do this when you confirm)
- Implement the local header `gpio_coproc_local.h` and change `gpio.c` to use the inline wrappers and re-run the build.
- If you want me to proceed now, confirm and I will start by creating `src/main_board/low_level/gpio_coproc_local.h` and updating `gpio.c` to use it. If you prefer to run the diagnostic commands first and paste outputs, I’ll analyze them before making edits.

Notes / caveats
- The `mcr` coprocessor assembly sequences are architecture-specific. If rp2350's Cortex-M33 core differs from the RP2040 approach, memory mapped SIO writes will be the fallback.
- Because you requested not to use SDK headers directly, I will only copy the minimal inline assembly wrappers into our local header — no other SDK code will be referenced.
- Keep a short audit comment in the local header that notes the code was adapted (source: SDK-style wrapper), and state which SDK version you copied from for traceability.

Timeline estimate
- Diagnostics & initial header + small edits: 30–90 minutes.
- Rebuild + debug: another 15–60 minutes depending on issues found.
- UF2 conversion and flash verification: 15–45 minutes.

End of TODO. Make sure to commit changes to the repo and push a branch like `fix/gpio-fast-op-local` for traceability.