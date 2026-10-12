# Changelog

`virtual-bus` and `virtual-bus-build` share one version: the generated bindings of one must match the
runtime of the other. Before 1.0, a breaking change raises the minor version.

## 0.5.1 (2026-10-12)

No code changes since 0.5.0.

### Changed

- The crates.io README, the package description and the crate docs open with what the crate is for:
  testing embedded Rust drivers without hardware, from register maps to RTL simulation. The crates.io
  README adds a short list of benefits and an example (2451272)

### Repository

- README lists the crates.io drivers tested with virtual-bus 0.5.0
- The MCP23017 example's tests are split into the model on its own (`mcp23017_model.rs`) and the
  crates.io driver on both paths (`mcp23017_driver.rs`)
- More tests: I2C data NACK on the signal lines, faults through a crates.io driver and recovery, one
  driver on both SPI paths, `SimPin` edges, and the whole interrupt flow of the `spi_counter` RTL,
  delays due at the same time, and several tasks waiting on one `SimPin`
- READMEs: dependencies to add; `SimClock`, `SimPin` and `block_on`

## 0.5.0 (2026-10-11)

### Breaking changes

- **virtual-bus**: removed `SimSpiBus::set_min_cs_high_ns` and `cs_high_violations`, the only timing check
  on the buses. Timing margins are checked on real hardware; a model that needs a minimum CS high time
  can compare the time of `select` with that of the last `deselect` itself. The gap the master leaves
  between frames (`set_cs_high_ns`) stays (7cdeb63)
- **virtual-bus**: the async `SimDelay` stops the task until its clock has moved on, like `#delay` in
  SystemVerilog, instead of moving the clock at once. `block_on` moves the clock to the next deadline when
  nothing can run, so tasks joined together run in time order. Async code that awaited a delay outside
  `block_on` (polling by hand) now sees `Pending`. The blocking `DelayNs` still moves the clock at once
  (f7b6423)

### Added

- **virtual-bus**: `SimI2cBus::with_clock` / `SimSpiBus::with_clock` and `clock()`. Signal lines can share
  one `SimClock` with transaction-level models: moving it from anywhere (a delay, a model's `advance_ns`,
  another bus) runs the system clocks of the models on every line that follows it (36c4b22)
- **virtual-bus**: `SimPin`, a wire from a chip (IRQ, DRDY) that tests and models drive and drivers read
  (`InputPin`) or wait on (`digital::Wait`). A wait sees only the edges after it starts, as on a real HAL
  (f7b6423)
- **virtual-bus**: `block_on` is a scheduler: wakes a `SimPin` waiter, moves time for a waiting delay,
  and drops the timer of a delay that was dropped (a timeout that lost a `select`) (f7b6423)
- Repository: `examples/rtl-demo/tests/shared_time.rs` times an EEPROM model and the `spi_counter` RTL
  with one `SimClock`, and waits for the RTL's interrupt from an async task (898302e)

## 0.4.0 (2026-10-11)

### Breaking changes

- **virtual-bus**: `VirtualI2cBus` calls `I2cSlave::write` one byte at a time, as the signal lines
  already did. After a model returns `Err(Nack)`, the rest of the write is not sent (UM10204: the
  master stops after a NACK). Models that expected the whole write in one call must collect the bytes
  between `start` and `stop`. The signature is unchanged (c279dc4)
- **virtual-bus**: an empty write (address probe) no longer calls `write` (c279dc4)

### Added

- **virtual-bus**: embedded-hal-async. `VirtualI2cBus` and `BitBangI2c` implement the async `I2c`,
  `VirtualSpiDevice` the async `SpiDevice`, `VirtualSpiBus` and `BitBangSpi` the async `SpiBus`, and
  `SimDelay` the async `DelayNs`. Each calls its blocking twin, so the log and fault injection are the
  same and every future finishes on the first poll (2992eea)
- **virtual-bus**: `block_on`, to call async drivers from a plain `#[test]`. A future that wakes itself
  (`yield_now`) is polled again; one that returns `Pending` with nothing to wake it panics instead of
  hanging the test (2992eea, 5f7e7d5)
- **virtual-bus**: `SimClock`, simulated time for models on the transaction-level buses. Models read
  `now_ns`; drivers move it with `delay()` (a `SimDelay`, blocking and async). The bus itself takes no time.
  Time is kept in ps, like the signal lines (561f131, 7439848)
- **virtual-bus**: a test running `lis3dh-async` from crates.io on a LIS3DH register map, over I2C and SPI
  (5f7e7d5)

### Note

- Code that has both the blocking and the async trait in scope and calls a method on a bus with dot
  syntax (`bus.transaction(..)`) no longer compiles (E0034, multiple applicable items). Call it as
  `embedded_hal::i2c::I2c::transaction(&mut bus, ..)`, or drop a hand-written async adapter: the bus
  is now async itself (2992eea)

## 0.3.1 (2026-10-11)

First release on crates.io. No code changes since 0.3.0.

### Changed

- Package metadata for crates.io: `repository`, `keywords` and `categories`
- README links point to GitHub and docs.rs so they work on crates.io; the `virtual-bus-build` README adds
  the dependencies from crates.io instead of the git tag

## 0.3.0 (2026-10-10)

### Breaking changes

- **virtual-bus**: `I2cSlave::start` returns `Result<(), Nack>`, so a model can NACK its own address
  (an EEPROM during its write cycle, a sensor still measuring). Models that override `start` must
  return `Ok(())`; models that don't are unchanged (5b09025)

### Added

- Address NACK from a model, on both `VirtualI2cBus` and `PinLevelI2cSlave`. `stop` is called only if
  some `start` in the transaction was ACKed, so a write followed by a NACKed repeated START still gets
  its `stop` (5b09025)

### Fixed

- **virtual-bus-build**: an unpacked array port on the top (`input logic [7:0] a [4]`) was left out of
  the model without a word, so it stayed at 0. It now stops the build with a message, like `inout` and
  wider ports (a0d9780)

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

- From `examples/`: the system-clock I2C slave, `spi_whoami.v` and its chip top, the hand-ported SPI
  pin model, and `marlin-demo`. One I2C and one SPI sample remain; the I2C chip top (`inout` pad,
  power-on reset) is kept, now around the SCL-only slave (b1959da and the commit after it)

## 0.1.0 (2026-10-05)

First version: virtual I2C / SPI buses for embedded-hal 1.0 drivers, bit-bang masters, simulated signal
lines, `RegisterMap`, fault injection, and Verilator models through `virtual-bus-build` (0310f90).
