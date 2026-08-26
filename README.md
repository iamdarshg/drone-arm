This is a codebase that is woring to create a hardware+software solution for dron control and management in a relatively modular manner. 
This is a codebase that is woring to create a hardware+software solution for dron control and management in a relatively modular manner.
This needs to be inherenly zero-trust, zero-fail and easy to analyse statically. The langauge of choice is C, with rust and python tooling accepted as well.Follow the 10 rules as set by nasa for code, the same should apply here due to the similar no-fail environment.
Currently, the main control board has been designed and the auxiallry field-oriented control esc needs to be designed.
Ideally, as much of this code as possible should be library free and written entirely from scratch. If any libraries are used beyond standard c libararies, they should all be stored within this folder, nowehere else.
Please add as much documentation as is needed, whenevr you refer to something just download and leave it in the docs folder. Please use the pico SDK exclusively as a library for C and/or write everything using reigster level functions.
Build will occur via Meson & Ninja. This means we will be compiling the whole thing using the arm toolchain, against the headers and flash tables and such from the SDK.
When debug is enabled in the meson build, we will output the assembly instead of the .uf2 file.

## Motor Platform (selected prototype baseline)

The ESC (Rev-B) is being optimized for the **Flycci FA4119 KV350** ([product page](https://robu.in/product/flycci-fpv-brushless-motors-fa4119-350kv/)). The current prototype decision, alternatives, cost model, proposed 550 Kv rewind, selected 8x4.5 propeller and mandatory validation gates are documented in [hardware/motor_release/FA4119_PROPULSION_DECISION.md](hardware/motor_release/FA4119_PROPULSION_DECISION.md).

- 12N14P outrunner (**7 pole pairs**), stator 41x19 mm, 280 g w/ cable
- Recommended 10S-12S LiPo, 13" prop, 80 A ESC class
- Measured (12S, HQ13x9-3 3-blade): 8.18 kg max thrust / 3.47 kW max @ 48 V, 74 A battery
- A rewind to **approximately 550 kV** is the experimental target if stock-motor dyno results justify it. The chosen Orange HD 8045 propeller has no verified manufacturer RPM rating in the repository, so the 550 Kv/12S combination requires closed-loop RPM limiting and guarded testing; it is not flight-released.

### eRPM headroom

eRPM = mech RPM x pole pairs (7). At 12S max (50.4 V):

| Winding | No-load RPM | No-load eRPM | Published or initially governed loaded eRPM |
|---|---|---|---|
| 350 kV (stock) | 17,640 | ~123 k | ~77-80 k with the manufacturer's 13-inch test prop |
| 550 kV (rewind) | 27,720 | ~194 k | initially <=91 k with the 8-inch prop (13,000 mechanical-rpm ceiling) |

**280 k eRPM check:** 280 k eRPM = 4.67 kHz electrical = 40,000 mechanical RPM at 7 pole pairs. This remains a possible future bare-rotor research point, not a propeller operating point. An 8-inch prop at 40,000 RPM has approximately 426 m/s rotational tip speed before pitch velocity and is unsuitable. The proposed 550 Kv winding reaches 194 k eRPM at fully charged 12S; initial propeller testing is capped at 13,000 mechanical RPM (91 k eRPM) pending a verified rating for the exact propeller.

## Build Instructions

This project uses Meson and Ninja for building.

### Prerequisites
- [ARM GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (`arm-none-eabi-gcc`)
- [Meson](https://mesonbuild.com/)
- [Ninja](https://ninja-build.com/)
- Python 3

### Configuration
Create a build directory and configure the project:
```bash
meson setup build
```

To enable **Debug Mode** (includes assembly output and `-O0`):
```bash
meson setup build -Ddebug=enabled 
```

### Building
Build the firmware:
```bash
ninja -C build
```
This produces `firmware_main_board.uf2` and `firmware_esc_module.bin` in the `build` directory.

### Testing
To build and run the unit tests:
```bash
meson test -C build
```
Current tests include:
- `test_main`: Scheduler and basic async operations.
- `test_clock_safety`: (New) Integrity protection for global clock state.
The main software will run as two non-synchronised loops- the sensor loop and the PWM loop. The one loop will run at the speed of the ESCs and will provide the best estimate for what the thrust should be at any given time to the ESCs, 60Hz if communicating via PWM and 1 kHz if communicating via SPI. This rate will be configurable. This loop may also be offloaded to the PIO modules in the future. The other loop is reponsible for sensor measuerment and fusion.
