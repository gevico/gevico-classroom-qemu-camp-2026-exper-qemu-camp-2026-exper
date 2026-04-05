<p align="center"><strong>QEMU Camp 2026 — Experiment Repository</strong></p>
<p align="center"><a href="README.md">English</a> | <a href="README_zh.md">中文</a></p>

This is the professional-stage experiment repository for QEMU Camp 2026. It covers four experiment directions, all based on RISC-V.

## Online Documentation

| Direction | Experiment Manual | Hardware Datasheet / Guide |
|-----------|------------------|----------------------------|
| **CPU** | [CPU Experiment Manual](https://qemu.gevico.online/exercise/2026/stage1/cpu/cpu-exper-manual/) | [CPU Datasheet](https://qemu.gevico.online/exercise/2026/stage1/cpu/cpu-datasheet/) |
| **SoC** | [SoC Experiment Manual](https://qemu.gevico.online/exercise/2026/stage1/soc/g233-exper-manual/) | [G233 SoC Datasheet](https://qemu.gevico.online/exercise/2026/stage1/soc/g233-datasheet/) |
| **GPGPU** | [GPU Experiment Manual](https://qemu.gevico.online/exercise/2026/stage1/gpu/gpu-exper-manual/) | [GPU Datasheet](https://qemu.gevico.online/exercise/2026/stage1/gpu/gpu-datasheet/) |
| **Rust** | [Rust Experiment Manual](https://qemu.gevico.online/exercise/2026/stage1/rust/rust-exper-manual/) | [Rust Programming Guide](https://qemu.gevico.online/exercise/2026/stage1/rust/rust-lang-manual/) |

Full tutorial site: <https://qemu.gevico.online/>

## Experiment Directions

| Direction | Test Framework | Test Location | Scoring |
|-----------|---------------|---------------|---------|
| **CPU** | TCG testcase | `tests/gevico/tcg/` | 10 tests x 10 pts = 100 |
| **SoC** | QTest | `tests/gevico/qtest/` | 10 tests x 10 pts = 100 |
| **GPGPU** | QTest (QOS) | `tests/qtest/gpgpu-test.c` | 17 tests -> 100 pts |
| **Rust** | QTest + unit | `tests/gevico/qtest/` + `rust/hw/i2c/` | 10 tests x 10 pts = 100 |

## Quick Start

### 1. Install Dependencies

```bash
# Ubuntu 24.04
sudo sed -i 's/^Types: deb$/Types: deb deb-src/' /etc/apt/sources.list.d/ubuntu.sources
sudo apt-get update
sudo apt-get build-dep -y qemu

# RISC-V bare-metal cross compiler (required for CPU experiment)
sudo mkdir -p /opt/riscv
wget -q https://github.com/riscv-collab/riscv-gnu-toolchain/releases/download/2025.09.28/riscv64-elf-ubuntu-24.04-gcc-nightly-2025.09.28-nightly.tar.xz -O riscv-toolchain.tar.xz
sudo tar -xJf riscv-toolchain.tar.xz -C /opt/riscv --strip-components=1
sudo chown -R $USER:$USER /opt/riscv
export PATH="/opt/riscv/bin:$PATH"
echo 'export PATH="/opt/riscv/bin:$PATH"' >> ~/.bashrc
riscv64-unknown-elf-gcc --version

# Rust toolchain (required for Rust experiment and build)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
. "$HOME/.cargo/env"

cargo install bindgen-cli
```

### 2. Configure

```bash
make -f Makefile.camp configure
```

This runs `./configure` with unified flags for all experiments:
- `--target-list=riscv64-softmmu,riscv64-linux-user`
- `--extra-cflags='-O0 -g3'`
- `--cross-prefix-riscv64=riscv64-unknown-elf-`
- `--enable-rust`

### 3. Build

```bash
make -f Makefile.camp build
```

### 4. Run Tests

```bash
# Run a specific experiment
make -f Makefile.camp test-cpu
make -f Makefile.camp test-soc
make -f Makefile.camp test-gpgpu
make -f Makefile.camp test-rust

# Run all experiments
make -f Makefile.camp test
```

### 5. Submit

```bash
git add .
git commit -m "feat: implement ..."
git push origin main
```

CI will automatically build, run tests, calculate scores, and upload to the ranking platform. Scores of 0 are not uploaded.

## Experiment Details

### CPU Experiment (TCG)

Implement custom RISC-V instructions for the G233 machine. Tests verify instruction behavior via semihosting-based bare-metal programs.

- Machine: `g233` (`hw/riscv/g233.c`)
- Tests: `tests/gevico/tcg/riscv64/test-insn-*.c`
- Run: `make -f Makefile.camp test-cpu`
- Docs: [Experiment Manual](https://qemu.gevico.online/exercise/2026/stage1/cpu/cpu-exper-manual/) | [CPU Datasheet](https://qemu.gevico.online/exercise/2026/stage1/cpu/cpu-datasheet/)

### SoC Experiment (QTest)

Implement peripheral device models (GPIO, PWM, WDT, SPI, Flash) for the G233 SoC. Tests verify register behavior and interrupt connectivity via QTest MMIO read/write.

- Peripherals: GPIO (`0x10012000`), PWM (`0x10015000`), WDT (`0x10010000`), SPI (`0x10018000`)
- Tests: `tests/gevico/qtest/test-*.c`
- Run: `make -f Makefile.camp test-soc`
- Docs: [Experiment Manual](https://qemu.gevico.online/exercise/2026/stage1/soc/g233-exper-manual/) | [G233 SoC Datasheet](https://qemu.gevico.online/exercise/2026/stage1/soc/g233-datasheet/)

### GPGPU Experiment (QTest)

Implement a PCIe GPGPU device with SIMT execution engine, DMA, and low-precision float support. Tests verify device registers, VRAM, kernel execution, and FP8/FP4 conversions.

- Device: `hw/gpgpu/` (PCI device `gpgpu`)
- Tests: `tests/qtest/gpgpu-test.c` (17 subtests)
- Run: `make -f Makefile.camp test-gpgpu`
- Docs: [Experiment Manual](https://qemu.gevico.online/exercise/2026/stage1/gpu/gpu-exper-manual/) | [GPU Datasheet](https://qemu.gevico.online/exercise/2026/stage1/gpu/gpu-datasheet/)

### Rust Experiment (QTest + Unit)

Implement I2C bus, GPIO I2C controller, and SPI controller in Rust for the G233 SoC. Unit tests verify core Rust logic; QTest tests verify device register behavior and peripheral communication (AT24C02 EEPROM over I2C, AT25 EEPROM over SPI).

- I2C bus: `rust/hw/i2c/src/lib.rs` (3 unit tests)
- GPIO I2C controller: base `0x10013000`, connected AT24C02 EEPROM (addr `0x50`)
- SPI controller: base `0x10019000`, connected AT25 EEPROM
- Tests: `tests/gevico/qtest/test-i2c-*.c`, `tests/gevico/qtest/test-spi-rust-*.c`
- Run: `make -f Makefile.camp test-rust`
- Docs: [Experiment Manual](https://qemu.gevico.online/exercise/2026/stage1/rust/rust-exper-manual/) | [Rust Programming Guide](https://qemu.gevico.online/exercise/2026/stage1/rust/rust-lang-manual/)


## Available Make Targets

```
make -f Makefile.camp help       # Show all targets
make -f Makefile.camp configure  # Configure QEMU
make -f Makefile.camp build      # Build QEMU
make -f Makefile.camp test-cpu   # CPU experiment tests
make -f Makefile.camp test-soc   # SoC experiment tests
make -f Makefile.camp test-gpgpu # GPGPU experiment tests
make -f Makefile.camp test-rust  # Rust experiment tests
make -f Makefile.camp test       # All tests
make -f Makefile.camp clean      # Clean build
make -f Makefile.camp distclean  # Remove build directory
```

## Scoring

- Tests that **fail** do not break CI — they simply result in a lower score.
- Scores of **0** are not uploaded to the ranking platform.
- Each push to `main` triggers a full CI run.
