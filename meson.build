project('drone-arm', 'c',
  version: '0.2.0',
  default_options: [
    'c_std=c11',
    'warning_level=3',
    'werror=true',
  ]
)

src_inc = include_directories('src')
py = find_program('python3', required: true)

common_c_args = [
  '-Wall',
  '-Wextra',
  '-Werror',
]

test_uf2 = executable('test_uf2',
  ['tests/test_uf2.c', 'src/hal/uf2.c'],
  include_directories: [src_inc],
  c_args: common_c_args + ['-DTEST_HOST=1', '-DNDEBUG', '-O0', '-g'],
  native: true,
  install: false,
)

test('uf2_struct_test', test_uf2)

test('pad_checksum_test', py,
  args: [meson.project_source_root() + '/tests/test_pad_checksum.py'],
)

test('uf2_artifact_test', py,
  args: [meson.project_source_root() + '/tests/test_build_artifacts.py'],
)

test_scheduler = executable('test_scheduler',
  ['tests/test_scheduler.c', 'src/kernel/scheduler.c'],
  include_directories: [src_inc],
  c_args: common_c_args + ['-DNDEBUG', '-O0', '-g'],
  native: true,
  install: false,
)

test('scheduler_test', test_scheduler)

test_state_vector = executable('test_state_vector',
  ['tests/test_state_vector.c', 'src/sensors/state_vector.c'],
  include_directories: [src_inc],
  c_args: common_c_args + ['-DNDEBUG', '-O0', '-g'],
  native: true,
  install: false,
)

test('state_vector_test', test_state_vector)

test_clock_safety_sources = [
  'tests/test_clock_safety.c',
  'src/drivers/clock.c',
]

test_clock_safety = executable('test_clock_safety',
  test_clock_safety_sources,
  include_directories: [src_inc],
  c_args: common_c_args + ['-DNDEBUG', '-O0', '-g'],
  native: true,
  install: false,
)

test('clock_safety_test', test_clock_safety)

test_pwm_output = executable('test_pwm_output',
  [
    'tests/test_pwm_output.c',
    'src/pwm_output.c',
    'tests/gpio_stub.c',
  ],
  include_directories: [src_inc],
  c_args: common_c_args + ['-DTEST_HOST=1', '-DNDEBUG', '-O0', '-g'],
  native: true,
  install: false,
)

test('pwm_output_test', test_pwm_output)

test_driver_math = executable('test_driver_math',
  [
    'tests/test_driver_math.c',
    'src/drivers/dma.c',
    'src/drivers/uart.c',
    'src/drivers/spi.c',
    'src/drivers/i2c.c',
  ],
  include_directories: [src_inc],
  c_args: common_c_args + ['-DNDEBUG', '-O0', '-g'],
  native: true,
  install: false,
)

test('driver_math_test', test_driver_math)

if meson.is_cross_build()
  boot2 = executable('boot2.elf',
    'src/boot2.S',
    c_args: common_c_args,
    link_args: [
      '-nostdlib',
      '-Wl,--build-id=none',
      '-T' + meson.project_source_root() + '/src/boot2.ld',
    ],
    install: false,
  )

  boot2_bin = custom_target('boot2.bin',
    input: boot2,
    output: 'boot2.bin',
    command: ['arm-none-eabi-objcopy', '-O', 'binary', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
  )

  boot2_padded = custom_target('boot2_padded.S',
    input: boot2_bin,
    output: 'boot2_padded.S',
    command: [py, meson.project_source_root() + '/tools/pad_checksum', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
  )

  firmware = executable('firmware.elf',
    [
      'src/startup.S',
      'src/main.c',
      'src/board.c',
      'src/drivers/clock.c',
      'src/drivers/gpio.c',
      'src/drivers/uart.c',
      'src/drivers/i2c.c',
      'src/drivers/spi.c',
      'src/drivers/dma.c',
      'src/system/multicore.c',
      'src/kernel/scheduler.c',
      'src/kernel/assert.c',
      'src/pio.c',
      'src/pwm_output.c',
      'src/sensors/state_vector.c',
      boot2_padded,
    ],
    include_directories: [src_inc],
    c_args: common_c_args + ['-ffunction-sections', '-fdata-sections', '-fno-builtin'],
    link_args: [
      '-nostdlib',
      '--specs=nosys.specs',
      '--specs=nano.specs',
      '-Wl,--gc-sections',
      '-Wl,--build-id=none',
      '-T' + meson.project_source_root() + '/src/linker.ld',
    ],
    install: false,
  )

  firmware_bin = custom_target('firmware.bin',
    input: firmware,
    output: 'firmware.bin',
    command: ['arm-none-eabi-objcopy', '-O', 'binary', '@INPUT@', '@OUTPUT@'],
    build_by_default: true,
  )

  custom_target('firmware.uf2',
    input: firmware,
    output: 'firmware.uf2',
    command: [py, meson.project_source_root() + '/tools/uf2conv.py', '@INPUT@', '-o', '@OUTPUT@', '-f', 'rp2350-arm'],
    build_by_default: true,
  )
endif
