# virtual-bus-build

Attach your own Verilog to [virtual-bus](../README.md): this crate runs Verilator from `build.rs` and
generates a Rust `Model` per RTL, with a method per port. You then write a small adapter that drives the
model's ports from the simulated signal lines, and test the RTL with ordinary embedded-hal drivers.

This page walks through the whole flow. The working sample is `examples/rtl-demo`
(`examples/verilog/rtl/` for the RTL); try it with `cargo run -p rtl-demo --example spi_counter`.

```text
your RTL ──build.rs──▶ Model (set_<input>, <output>(), eval) ──adapter──▶ I2cPinModel / SpiPinModel
                                                                              │
              embedded-hal driver ──▶ BitBangI2c / BitBangSpi ──▶ SimI2cBus / SimSpiBus
```

## Requirements

- Verilator 5.x (`verilator` on `PATH`, or `VERILATOR_ROOT`) and a C++17 compiler
- Tested on Linux (including WSL2) with Verilator 5.052

## 1. Add the dependencies

Both crates come from this repository. `virtual-bus` is needed too: the generated code refers to it.

```toml
[dependencies]
virtual-bus = { git = "https://github.com/Nagi-905/virtual-bus", tag = "v0.2.0" }
embedded-hal = "1.0"

[build-dependencies]
virtual-bus-build = { git = "https://github.com/Nagi-905/virtual-bus", tag = "v0.2.0" }

[dev-dependencies]
embedded-hal-bus = { version = "0.3", features = ["std"] }   # ExclusiveDevice for SPI
```

## 2. Prepare the RTL

- **Top-level ports are inputs and outputs of 1 to 64 bits.** `inout` ports, wider ports and
  unpacked array ports (`input logic [7:0] a [4]`) stop the build with a message; Verilator itself
  rejects interface ports on the top. Packed structs and enums are fine: they become plain integers
- **SystemVerilog works too.** List `.sv` files (packages first) the same way as `.v` files
- **Split bidirectional pins.** SDA becomes an input and a "pull low" output (`sda_i`, `sda_low`);
  MISO becomes an output and an output enable (`miso`, `miso_oe`). virtual-bus resolves the wires
  (open drain, pull-ups, contention) on the Rust side
- **If your chip top has `inout` pads**, wrap it in a simulation-only module that resolves the line and
  exposes only inputs and outputs. Its adapter returns `true` from `I2cPinModel::wants_external_sda`,
  so that it gets the SDA level driven by everyone else:

  ```verilog
  module my_chip_sim (input wire scl, input wire ext_sda_low, output wire sda_level /* , ... */);
    tri1 sda;                                // pull-up
    assign sda = ext_sda_low ? 1'b0 : 1'bz;  // someone outside pulls SDA low
    my_chip dut (.scl(scl), .sda(sda) /* , ... */);
    assign sda_level = sda;
  endmodule
  ```

  The complete example is `examples/verilog/rtl/i2c_whoami_top.v`, its wrapper `sim/i2c_whoami_sim.v` and
  the adapter `VerilatedWhoAmITop` in `examples/rtl-demo/src/i2c_whoami.rs`

- **Power pins are ordinary inputs.** A `vdd` input that drives a power-on reset inside the RTL can be
  toggled from a test like any other port (`set_vdd(false)`, `eval`, `set_vdd(true)`); see the
  power-cycle tests in `examples/rtl-demo/src/i2c_whoami.rs`
- Lint first: `verilator --lint-only -Wall rtl/my_chip.v --top-module my_chip`

## 3. List the RTL in `build.rs`

```rust
use virtual_bus_build::{Model, Verilated};

fn main() {
    Verilated::new()
        .rtl_dir("rtl") // relative to the crate root
        .model(
            // module name in Rust (lowercase, digits, _), then the Verilog top module
            Model::new("my_chip", "my_chip")
                .source("my_chip.v")
                .source("my_chip_regs.v")
                .flag("-Wall")
                .trace(), // only if you want VCD waveforms
        )
        .compile();
}
```

Each model is cached in `OUT_DIR` and rebuilt only when its settings or the content of a file Verilator
read for it changes, so editing one RTL file rebuilds only the models that use it.

## 4. Include the generated code

```rust
// src/lib.rs
pub mod bindings {
    include!(concat!(env!("OUT_DIR"), "/verilated_models.rs"));
}
```

Each model gets a module (`bindings::my_chip`) with a `Model`:

| Port | Generated |
|---|---|
| input `scl` | `set_scl(v)`; takes effect at the next `eval()` |
| output `sda_low` | `sda_low()` |
| width | 1 bit is `bool`; wider ports are `u8` / `u16` / `u32` / `u64` |
| always | `new()`, `eval()`, `set_time_ps`, `eval_before`, `is_tracing` |
| with `.trace()` | `open_vcd(path)`, `close_vcd()` |

`cargo doc --open` shows the generated methods under your crate's `bindings` module.

## 5. Write the adapter

The adapter owns the `Model` and implements `I2cPinModel` or `SpiPinModel` from virtual-bus.
What to write depends on how the RTL is clocked.

| Method | RTL driven by SCL / SCK only | RTL on a system clock |
|---|---|---|
| `set_inputs(..)` | set the inputs, then `eval()` | set the inputs only; `eval()` too if `is_tracing()` |
| `period_ps()` | leave the default (`None`) | `Some(period)`, e.g. `Some(20_000)` for 50 MHz |
| `tick()` | not called | `clk` 0 → `eval_before(period / 2)` → 1 → `eval()` |
| `sda_low()` / `miso()` | read the outputs (`miso_oe` true → `Some(miso)`, else `None`) | same |
| `set_time_ps(t)` | forward to the model if you write VCDs | same |

```rust
use virtual_bus::bus::spi::sim::SpiPinModel;
use crate::bindings::my_chip;

pub struct MyChip {
    m: my_chip::Model,
}

impl MyChip {
    pub const PERIOD_PS: u64 = 20_000; // 50 MHz

    pub fn new() -> Self {
        let mut m = my_chip::Model::new();
        m.set_cs_n(true);
        // Verilator starts every input at 0: go 1 → 0 → 1 to give the asynchronous reset an edge
        for level in [true, false, true] {
            m.set_rst_n(level);
            m.eval();
        }
        Self { m }
    }

    /// Outputs that are not bus pins get their own methods
    pub fn irq(&self) -> bool {
        self.m.irq()
    }
}

impl SpiPinModel for MyChip {
    fn set_inputs(&mut self, cs_n: bool, sck: bool, mosi: bool) {
        self.m.set_cs_n(cs_n);
        self.m.set_sck(sck);
        self.m.set_mosi(mosi);
        // the state only moves on clk; evaluate now only so the VCD shows the inputs on time
        if self.m.is_tracing() {
            self.m.eval();
        }
    }
    fn miso(&self) -> Option<bool> {
        self.m.miso_oe().then(|| self.m.miso())
    }
    fn period_ps(&self) -> Option<u64> {
        Some(Self::PERIOD_PS)
    }
    fn tick(&mut self) {
        self.m.set_clk(false);
        self.m.eval_before(Self::PERIOD_PS / 2); // the low half, half a period earlier in the VCD
        self.m.set_clk(true);
        self.m.eval();
    }
    fn set_time_ps(&mut self, now_ps: u64) {
        self.m.set_time_ps(now_ps);
    }
}
```

Full adapters: `examples/rtl-demo/src/i2c_whoami.rs` (SCL / SDA only, and a chip top with an `inout` pad)
and `examples/rtl-demo/src/spi_counter.rs` (system clock, `irq`, VCD).

## 6. Test it with a driver

```rust
use embedded_hal::spi::{MODE_0, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use virtual_bus::bus::spi::sim::SimSpiBus;
use virtual_bus::shared;

let bus = SimSpiBus::new();
let dut = shared(MyChip::new());           // keep a handle to read irq()
let cs = bus.add_device(dut.clone());
let spi = bus.master(MODE_0, 1_000_000).unwrap();
let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();

let mut b = [0x80, 0x00];                  // command and read in one CS frame
dev.transfer_in_place(&mut b).unwrap();
bus.run_ns(10_000);                        // let simulated time pass
assert!(dut.borrow().irq());
```

- Time is simulated: it moves only with `delay_ns` / `run_ns` and while the master clocks bits.
  A 1 MHz transfer of two bytes takes about 17 µs, and an RTL on a system clock keeps running meanwhile
- An RTL that samples SCK with its own clock needs SCK well below that clock (about clk / 8)
- The bus keeps CS high for one SCK period between frames, so a slave that synchronizes CS sees each
  frame end. If you build a `BitBangSpi` from the pins yourself, call `SimSpiBus::set_cs_high_ns`
- Running the same tests on a Rust model (`RegisterMap` or your own `I2cSlave` / `SpiSlave`) and on the RTL
  is a quick way to check the RTL against a spec: see `examples/rtl-demo/tests/i2c_conformance.rs`

## 7. When it does not work

Read a fixed register (an ID) first. If that fails, suspect the adapter. Then build the model with
`.trace()`, open a VCD (`Model::open_vcd`) and look at it in GTKWave.

| In the VCD | Likely cause |
|---|---|
| Nothing moves, not even `clk` | `set_time_ps` is not forwarded to the model |
| `clk` never toggles | `period_ps` is not implemented, so `tick` is never called |
| `clk` moves but `sck` is flat | `set_inputs` writes SCK to the wrong port |
| `cs_n` stays low across several transfers | no CS gap: the master was built by hand without `set_cs_high_ns` |
| Read data shifted by one bit | SCK too fast for an RTL that samples it with its own clock |
| The file is empty or cut short | the model is still alive: drop the bus, its pins and delays first |
