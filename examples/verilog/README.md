# Sample Verilog

RTL that `examples/rtl-demo` turns into models with Verilator.

| File | Contents |
|---|---|
| `rtl/i2c_whoami.v` | I2C slave with two registers. No system clock: runs on SCL / SDA only |
| `rtl/i2c_whoami_top.v` | The same I2C slave as a chip top: SDA is an `inout` pad without output enable, and a power-on reset comes from `vdd` |
| `rtl/sim/i2c_whoami_sim.v` | Simulation-only wrapper around the chip top: resolves SDA with a pull-up (`tri1`) and exposes only inputs and outputs |
| `rtl/spi_counter.v` | SPI timer / counter. Runs on a system clock and samples SCK / CS_N / MOSI with it; has an `irq` output |
| `tb/` | Icarus Verilog unit testbench for `i2c_whoami.v` and, with `-DUSE_TOP`, for the chip top |

The cores do not use `inout`: SDA is split into `sda_i` / `sda_low`, and MISO into `miso` / `miso_oe`.
virtual-bus-build accepts only inputs and outputs, so a chip top with `inout` pads is wrapped in a
simulation-only module, as `sim/i2c_whoami_sim.v` does for `i2c_whoami_top.v`.

```sh
make lint   # lint with Verilator
make sim    # run the unit tests with Icarus Verilog
```
