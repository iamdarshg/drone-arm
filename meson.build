project('drone-arm', 'c',
  version: '0.2.0',
  default_options: [
    'c_std=c11',
    'warning_level=2',
    'werror=true',
  ]
)

src_inc = include_directories('src')

host_test_src = [
  'tests/test_uf2.c',
  'src/hal/uf2.c',
]

test_uf2 = executable('test_uf2',
  host_test_src,
  include_directories: [src_inc],
  c_args: ['-DTEST_HOST=1', '-DNDEBUG', '-O0', '-g'],
  install: false,
)

test('uf2_struct_test', test_uf2)

py = find_program('python3', required: true)

test('uf2_artifact_test', py,
  args: ['tests/test_build_artifacts.py'],
)
