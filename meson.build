project('drone-arm', 'c',
  version: '0.1.0',
  default_options: [
    'c_std=c11',
    'warning_level=2',
    'werror=true',
  ]
)

arm_toolchain_config = configuration_data()
arm_toolchain_config.set('arch', 'arm')
arm_toolchain_config.set('cpu', 'cortex-m33')
arm_toolchain_config.set('fpu', 'fpv5-d16')
arm_toolchain_config.set('float_abi', 'hard')
arm_toolchain_config.set('thumb', true)

arm_args = [
  '-mcpu=@0@'.format(arm_toolchain_config.get('cpu')),
  '-mthumb',
  '-mfloat-abi=@0@'.format(arm_toolchain_config.get('float_abi')),
  '-mfpu=@0@'.format(arm_toolchain_config.get('fpu')),
  '-march=armv8m.main+fp+dp',
]

add_project_arguments(arm_args, language: 'c')
add_project_link_arguments(arm_args, language: 'c')

pico_sdk_inc = include_directories('include')

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

debug_mode = get_option('debug')
if debug_mode.enabled()
  add_project_arguments('-g', '-O0', '-DDEBUG=1', language: 'c')
  message('DEBUG mode: outputting .asm files instead of .uf2')
else
  add_project_arguments('-O2', '-DNDEBUG=1', language: 'c')
endif

common_inc = include_directories('src/common')

common_sources = [
  'src/common/math_utils.c',
  'src/common/filter.c',
  'src/common/ring_buffer.c',
  'src/common/crc.c',
  'src/common/watchdog.c',
]

main_board_inc = include_directories('src/main_board', 'src/main_board/hardware_defs', 'src/main_board/low_level')

main_board_sources = [
  'src/main_board/main.c',
  'src/main_board/init.c',
  'src/main_board/irq.c',
]

esc_inc = include_directories('src/esc_controller')

esc_sources = [
  'src/esc_controller/esc.c',
  'src/esc_controller/foc.c',
  'src/esc_controller/pwm.c',
  'src/esc_controller/encoder.c',
  'src/esc_controller/current_sense.c',
  'src/esc_controller/config.c',
]

link_args = [
  '--specs=nosys.specs',
  '--specs=nano.specs',
  '-lc',
  '-lm',
  '-lnosys',
]

all_sdk_inc = [
  pico_sdk_inc,
  pico_platform_inc,
  pico_hw_inc,
  pico_adc_inc,
  pico_pwm_inc,
  pico_spi_inc,
  pico_i2c_inc,
  pico_uart_inc,
  pico_timer_inc,
  pico_dma_inc,
  pico_pio_inc,
  pico_irq_inc,
  pico_clocks_inc,
  pico_resets_inc,
  pico_flash_inc,
  pico_sync_inc,
  pico_time_inc,
  pico_util_inc,
]

message('==============================================')
message('Drone Arm Build System')
message('==============================================')
message('Target: RP2354B (Cortex-M33 @ 150MHz)')
message('Compiler: arm-none-eabi-gcc')
message('')
message('Output targets:')
message('  1. firmware_main_board.uf2 - Main control board')
message('  2. firmware_esc_module.bin - ESC controller')
message('==============================================')

firmware_main_board = executable('firmware_main_board.elf',
  main_board_sources + common_sources,
  include_directories: all_sdk_inc + [main_board_inc, common_inc],
  link_args: link_args,
  install: false,
)

firmware_esc = executable('firmware_esc_module.elf',
  esc_sources + common_sources,
  include_directories: all_sdk_inc + [esc_inc, common_inc],
  link_args: link_args,
  install: false,
)

custom_target('firmware_main_board.uf2',
  input: firmware_main_board,
  output: 'firmware_main_board.uf2',
  command: ['arm-none-eabi-objcopy', '-O', 'binary', '@INPUT@', '@OUTPUT@'],
  capture: true,
)

custom_target('firmware_esc_module.bin',
  input: firmware_esc,
  output: 'firmware_esc_module.bin',
  command: ['arm-none-eabi-objcopy', '-O', 'binary', '@INPUT@', '@OUTPUT@'],
  capture: true,
)

if debug_mode.enabled()
  custom_target('firmware_main_board.lst',
    input: firmware_main_board,
    output: 'firmware_main_board.lst',
    command: ['arm-none-eabi-objdump', '-d', '-S', '@INPUT@', '>', '@OUTPUT@'],
    capture: true,
  )
  
  custom_target('firmware_esc_module.lst',
    input: firmware_esc,
    output: 'firmware_esc_module.lst',
    command: ['arm-none-eabi-objdump', '-d', '-S', '@INPUT@', '>', '@OUTPUT@'],
    capture: true,
  )
endif
