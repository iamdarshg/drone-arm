project('drone-arm', 'c',
  version: '0.1.0',
  default_options: [
    'c_std=c11',
    'warning_level=2',
    'werror=true',
  ]
)

# ARM toolchain configuration
arm_toolchain_config = configuration_data()
arm_toolchain_config.set('arch', 'arm')
arm_toolchain_config.set('cpu', 'cortex-m33')
arm_toolchain_config.set('fpu', 'fpv5-d16')
arm_toolchain_config.set('float_abi', 'hard')
arm_toolchain_config.set('thumb', true)

# ARM compiler flags
arm_args = [
  '-mcpu=@0@'.format(arm_toolchain_config.get('cpu')),
  '-mthumb',
  '-mfloat-abi=@0@'.format(arm_toolchain_config.get('float_abi')),
  '-mfpu=@0@'.format(arm_toolchain_config.get('fpu')),
  '-march=armv8m.main+fp+dp',
]

add_project_arguments(arm_args, language: 'c')
add_project_link_arguments(arm_args, language: 'c')

# Include directories
pico_sdk_inc = include_directories('include')
hardware_inc = include_directories('include/hardware')
common_inc = include_directories('src/common')
main_board_inc = include_directories('src/main_board', 'src/main_board/hardware_defs', 'src/main_board/low_level')
esc_inc = include_directories('src/esc_controller')

# SDK version defines
sdk_args = [
  '-DPICO_SDK_VERSION_MAJOR=2',
  '-DPICO_SDK_VERSION_MINOR=1',
  '-DPICO_SDK_VERSION_REVISION=0',
  '-DPICO_SDK_VERSION_STRING="2.1.0"',
  '-DRP2350',
  '-DPICO_RP2350A',
  '-DPICO_RP2350B',
  '-DPICO_SDK',
  '-DPICO_ON_DEVICE=1',
]
add_project_arguments(sdk_args, language: 'c')

# Debug/Release configuration
debug_mode = get_option('debug')
if debug_mode.enabled()
  add_project_arguments('-g', '-O0', '-DDEBUG=1', language: 'c')
  message('DEBUG mode: Building with debug symbols')
else
  add_project_arguments('-O2', '-DNDEBUG=1', language: 'c')
endif

# ============================================
# Boot Stage 2 Assembly
# ============================================

boot2_src = 'src/main_board/low_level/boot_stage2/boot2_w25q080.S'
boot2_ld = 'src/main_board/low_level/boot_stage2/boot_stage2.ld'

# Build boot stage 2 as binary
boot2_elf = custom_target('boot2_elf',
  input: boot2_src,
  output: 'boot2.elf',
  command: [
    'arm-none-eabi-gcc',
    '-mcpu=cortex-m33', '-mthumb',
    '-c', '@INPUT@',
    '-o', '@OUTPUT@',
    '-T', boot2_ld,
    '-I', 'src/main_board/low_level/boot_stage2',
  ],
  build_by_default: true,
)

boot2_bin = custom_target('boot2_bin',
  input: boot2_elf,
  output: 'boot2.bin',
  command: ['arm-none-eabi-objcopy', '-O', 'binary', '@INPUT@', '@OUTPUT@'],
  build_by_default: true,
)

# Pad and add checksum to boot2
boot2_padded = custom_target('boot2_padded',
  input: boot2_bin,
  output: 'boot2_padded.S',
  command: [
    python, 'tools/pad_checksum',
    '@INPUT@', '@OUTPUT@',
  ],
  build_by_default: true,
)

# ============================================
# Source Files
# ============================================

common_sources = [
  'src/common/math_utils.c',
  'src/common/filter.c',
  'src/common/ring_buffer.c',
  'src/common/crc.c',
  'src/common/watchdog.c',
  'src/common/scheduler.c',
]

main_board_sources = [
  'src/main_board/main.c',
  'src/main_board/init.c',
  'src/main_board/init_clocks.c',
  'src/main_board/irq.c',
  # Low level drivers
  'src/main_board/low_level/gpio.c',
  'src/main_board/low_level/spi.c',
  'src/main_board/low_level/i2c.c',
  'src/main_board/low_level/spi_irq.c',
  'src/main_board/low_level/adc.c',
  'src/main_board/low_level/timer.c',
  'src/main_board/low_level/pwm.c',
  'src/main_board/low_level/dma.c',
  # Boot stage 2 padded
  boot2_padded,
]

esc_sources = [
  'src/esc_controller/esc.c',
  'src/esc_controller/foc.c',
  'src/esc_controller/pwm.c',
  'src/esc_controller/encoder.c',
  'src/esc_controller/current_sense.c',
  'src/esc_controller/config.c',
]

# ============================================
# Linker Script
# ============================================

link_script = meson.current_source_dir() / 'src/main_board/linker_script.ld'

link_args = [
  '--specs=nosys.specs',
  '--specs=nano.specs',
  '-T' + link_script,
  '-lc',
  '-lm',
  '-lnosys',
]

# ============================================
# Build Messages
# ============================================

message('==============================================')
message('Drone Arm Build System')
message('==============================================')
message('Target: RP2354B (Cortex-M33 @ 150MHz+)')
message('Compiler: arm-none-eabi-gcc')
message('')
message('Features:')
message('  - Boot Stage 2 (W25Q080 flash support)')
message('  - Auto-overclocking to 95% max stable freq')
message('  - Cooperative scheduler with async/await')
message('  - SPI/I2C with FIFO interrupt handling')
message('')
message('Output targets:')
message('  1. firmware_main_board.uf2 - Main control board')
message('  2. firmware_esc_module.bin - ESC controller')
message('==============================================')

# ============================================
# Executables
# ============================================

firmware_main_board = executable('firmware_main_board.elf',
  main_board_sources + common_sources,
  include_directories: [hardware_inc, pico_sdk_inc, main_board_inc, common_inc],
  link_args: link_args,
  install: false,
)

firmware_esc = executable('firmware_esc_module.elf',
  esc_sources + common_sources,
  include_directories: [hardware_inc, pico_sdk_inc, esc_inc, common_inc],
  link_args: link_args,
  install: false,
)

# ============================================
# UF2 Conversion
# ============================================

python = find_program('python3', 'python')

uf2_main_board = custom_target('firmware_main_board.uf2',
  input: firmware_main_board,
  output: 'firmware_main_board.uf2',
  command: [
    python, 'tools/uf2conv.py',
    '@INPUT@',
    '-o', '@OUTPUT@',
    '-f', 'rp2350-arm-s',
  ],
  build_by_default: true,
)

bin_esc = custom_target('firmware_esc_module.bin',
  input: firmware_esc,
  output: 'firmware_esc_module.bin',
  command: ['arm-none-eabi-objcopy', '-O', 'binary', '@INPUT@', '@OUTPUT@'],
  build_by_default: true,
)

# ============================================
# Debug Output
# ============================================

if debug_mode.enabled()
  custom_target('firmware_main_board.lst',
    input: firmware_main_board,
    output: 'firmware_main_board.lst',
    command: ['arm-none-eabi-objdump', '-d', '-S', '@INPUT@'],
    capture: true,
    build_by_default: true,
  )
  
  custom_target('firmware_esc_module.lst',
    input: firmware_esc,
    output: 'firmware_esc_module.lst',
    command: ['arm-none-eabi-objdump', '-d', '-S', '@INPUT@'],
    capture: true,
    build_by_default: true,
  )
endif
