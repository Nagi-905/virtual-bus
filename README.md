# virtual-bus

A Rust crate for running embedded-hal 1.0 I2C / SPI drivers against IC models on your PC, without real hardware.

## Features

### Test hardware-targeted code on your PC, unchanged

The only boundary is the embedded-hal traits, so drivers and application code need no changes.
Drivers from crates.io work as they are. No board required: `cargo test` is enough to run it in CI.

### Run your own Verilog together with your firmware

List your RTL files in `build.rs`, and your Rust drivers can talk to the RTL through Verilator.
The ports are read from the RTL, and each becomes a typed method (`set_scl(true)`, `sda_low()`).
No C++ testbench to write. Check firmware and RTL together before the chip exists.

### Check a Rust model and the RTL against the same tests

Run the same driver and the same tests against a model written in Rust and against the RTL.
Write the spec quickly in Rust, then check mechanically that the RTL behaves the same way.
Both can also share one bus.

### Reproduce situations that are hard to create on real hardware

- Inject NACKs, bus errors, a stuck MISO or flipped bits to exercise your driver's error handling
- Catch SPI mode mismatches, a too-short CS idle time (t_CSH) and MISO contention
- With a power-on reset in your RTL, check that a power cycle brings registers back to their reset values

Time is simulated, so every run gives the same result, including timing-dependent bugs.

### Choose between speed and detail

Use the fast byte-level path most of the time, and switch to the pin-level path only when you need to check timing.
Both look the same to the driver.

```text
Driver (embedded-hal I2c / SpiDevice / SpiBus)
   ├─ transaction level  VirtualI2cBus / VirtualSpiDevice ── Rust model
   └─ pin level          BitBangI2c / BitBangSpi ── SimI2cBus / SimSpiBus ─┬─ Rust model
                                                                            └─ Verilog RTL
```

## Usage

### Build a model and attach it

```rust
use embedded_hal::i2c::I2c;
use virtual_bus::VirtualI2cBus;
use virtual_bus::devices::register::{I2cFormat, RegisterMap};

// Build a model from a list of registers and attach it at address 0x29
let regs = RegisterMap::new().ro(0x0F, 0xA5).rw(0x10, 0x00);
let mut bus = VirtualI2cBus::new();
bus.attach(0x29, regs.i2c(I2cFormat::new())).unwrap();

// Pass `bus` to your driver (here we write to it directly)
bus.write(0x29, &[0x10, 0x5A]).unwrap();
assert_eq!(regs.get(0x10), 0x5A);
```

### Run an existing driver

The virtual bus implements embedded-hal's `I2c`, so you can hand it straight to a driver from crates.io.
This example drives a hand-written MCP23017 model with the `mcp23017-driver` crate
(the model is in `virtual-bus/examples/mcp23017/model.rs`).

```rust
use embedded_hal::digital::OutputPin;
use mcp23017_driver::Mcp23017 as Driver;
use virtual_bus::{VirtualI2cBus, shared};

let model = shared(Mcp23017::new());     // a model implementing I2cSlave
let mut bus = VirtualI2cBus::new();
bus.attach(0x20, model.clone()).unwrap();

// The driver code is the same as on real hardware
let mut device = Driver::<_, 0x20>::new(&mut bus);
let (pins, _) = device.split().unwrap();
let mut led = pins.a0.into_push_pull_output().unwrap();
led.set_high().unwrap();

// Check that the driver's operation reached the model
assert!(model.borrow().pin(0));
```

```sh
cargo run -p virtual-bus --example mcp23017
```

For more, see the examples and the API documentation (`cargo doc --open`).

| Example | What it shows |
|---|---|
| `virtual-bus/examples/register_map.rs` | Attach a register-list model to virtual I2C and SPI buses |
| `virtual-bus/examples/pin_level.rs` | Drive the signal lines with bit-bang masters, and what happens when SPI modes don't match |
| `virtual-bus/examples/mcp23017/` | Write your own IC model and drive it with a driver from crates.io |
| `examples/rtl-demo/` | Turn the sample Verilog (`examples/verilog/`) into models with Verilator and attach them |
| `examples/marlin-demo/` | Load the same Verilog with [Marlin](https://github.com/ethanuppal/marlin) instead, and dump VCD waveforms |

```sh
cargo run -p virtual-bus --example register_map
cargo test --workspace     # also runs the RTL demos
cargo run -p marlin-demo --example i2c_wave    # writes target/i2c_whoami.vcd
```

## Crates

| Crate | Role |
|---|---|
| `virtual-bus` | Virtual buses, bit-bang masters, simulated signal lines, and building blocks for models |
| `virtual-bus-build` | Called from `build.rs`; turns Verilog into a model with Verilator and makes it usable from Rust |

## Requirements

- Rust 1.85 or later
- For Verilog models: Verilator 5.x and a C++17 compiler (5.025 or later for Marlin)

Tested on Linux (including WSL2).

## Limitations

- Only logical behavior is modeled. Check electrical characteristics and timing margins on real hardware
- I2C: clock stretching, arbitration, multi-master and 10-bit addresses are not supported
- SPI: only 4-wire SPI with separate MOSI and MISO. 3-wire and Dual / Quad SPI are not supported
- UART is not supported yet

## License

Licensed under the MIT License (`LICENSE`).
