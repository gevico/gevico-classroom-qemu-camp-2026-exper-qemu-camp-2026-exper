_libgevico_bindings_inc_rs = rust.bindgen(
  args: bindgen_args_common + [
    '--allowlist-file', meson.project_source_root() / 'include/hw/i2c/i2c.h',
    '--allowlist-file', meson.project_source_root() / 'include/hw/nvram/eeprom_at24c.h',
    '--blocklist-file',
      meson.project_source_root() /
      'include/(block\|chardev/|exec/|hw/core/|qemu/|qom/|system/).*',
  ],
  kwargs: bindgen_kwargs,
)

_libgevico_rs = static_library(
  'gevico_i2c_gpio',
  structured_sources(
    [
      'src/lib.rs',
      'src/bindings.rs',
    ],
    {'.' : _libgevico_bindings_inc_rs},
  ),
  override_options: ['rust_std=2021', 'build.rust_std=2021'],
  rust_abi: 'rust',
  link_with: [
    _util_rs,
    _migration_rs,
    _hwcore_rs,
  ],
  dependencies: [
    common_rs,
    libc_rs,
  ],
)

rust_devices_ss.add(when: 'CONFIG_X_I2C_RUST', if_true: [declare_dependency(
  link_whole: [_libgevico_rs],
  variables: {'crate': 'gevico_i2c_gpio'},
)])