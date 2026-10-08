# Changelog

`virtual-bus` and `virtual-bus-build` share one version: the generated bindings of one must match the
runtime of the other. Before 1.0, a breaking change raises the minor version.

## 0.2.0 (2026-10-08)

### Breaking changes

- **virtual-bus-build**: `Model::input` and `Model::output` are removed. The ports are read from the
  header Verilator generates, so drop the port list from `build.rs` (70e9c0c)
- **virtual-bus**: `verilated::VTable` has a new `trace` field. Code that builds a `VTable` by hand must
  set it (`None` for models built without `.trace()`) (3615bc8)
- **virtual-bus**: back-to-back SPI frames through a `SimSpiBus::master` are now one SCK period apart,
  instead of 0 ps. Simulated times after the first frame move later; tests that assert absolute times
  or count clock ticks across several frames need new expected values (8c85e64)

### Added

- Typed models: one `set_<input>` / `<output>()` method per RTL port, read from Verilator's output,
  including widths set by parameters (70e9c0c)
- VCD waveforms: build a model with `.trace()` and call `open_vcd`; the VCD follows the bus's simulated
  time, and any number of models can trace at once (3615bc8)
- `SimSpiBus::set_cs_high_ns`: the gap a master leaves between frames, for masters built from the pins
  (1e87f40)
- `examples/rtl-demo`: `spi_counter.v`, an SPI timer on a system clock with an `irq` output, its adapter
  and tests, and `cargo run -p rtl-demo --example spi_counter` (f332b56, fd67b40)
- `virtual-bus-build/README.md`: a step-by-step guide from RTL to a test (18603cc)

### Fixed

- A slave that synchronizes CS to a system clock saw back-to-back SPI frames as one, because CS was
  high for 0 ps between them (8c85e64)

### Removed

- From `examples/`: the system-clock I2C slave, `spi_whoami.v`, the chip tops without output enable and
  their simulation wrappers, the hand-ported SPI pin model, and `marlin-demo`. One I2C and one SPI
  sample remain (b1959da)

## 0.1.0 (2026-10-05)

First version: virtual I2C / SPI buses for embedded-hal 1.0 drivers, bit-bang masters, simulated signal
lines, `RegisterMap`, fault injection, and Verilator models through `virtual-bus-build` (0310f90).
