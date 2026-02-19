This is a codebase that is woring to create a hardware+software solution for dron control and management in a relatively modular manner. 
This is a codebase that is woring to create a hardware+software solution for dron control and management in a relatively modular manner.
This needs to be inherenly zero-trust, zero-fail and easy to analyse statically. The langauge of choice is C, with rust and python tooling accepted as well.Follow the 10 rules as set by nasa for code, the same should apply here due to the similar no-fail environment.
Currently, the main control board has been designed and the auxiallry field-oriented control esc needs to be designed.
Ideally, as much of this code as possible should be library free and written entirely from scratch. If any libraries are used beyond standard c libararies, they should all be stored within this folder, nowehere else.
Please add as much documentation as is needed, whenevr you refer to something just download and leave it in the docs folder. Please use the pico SDK exclusively as a library for C and/or write everything using reigster level functions.
Build will occur via Meson & Ninja. This means we will be compiling the whole thing using the arm toolchain, against the headers and flash tables and such from the SDK.
When debug is enabled in the meson build, we will output the assembly instead of the .uf2 file.

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
