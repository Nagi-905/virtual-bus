# Sample Verilog

RTL that `examples/rtl-demo` turns into models with Verilator.

| File | Contents |
|---|---|
| `rtl/i2c_whoami.v` | I2C slave with two registers. No system clock: runs on SCL / SDA only |
| `rtl/spi_counter.v` | SPI timer / counter. Runs on a system clock and samples SCK / CS_N / MOSI with it; has an `irq` output |
| `tb/` | Icarus Verilog unit testbench for `i2c_whoami.v` |

Neither uses `inout`: SDA is split into `sda_i` / `sda_low`, and MISO into `miso` / `miso_oe`.
virtual-bus-build accepts only inputs and outputs, so if your chip top has `inout` pins, wrap it in a
simulation-only module that splits them.

```sh
make lint   # lint with Verilator
make sim    # run the unit tests with Icarus Verilog
```
