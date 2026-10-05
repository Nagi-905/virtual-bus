# Sample Verilog

RTL that `examples/rtl-demo` turns into models with Verilator.

| File | Contents |
|---|---|
| `rtl/i2c_whoami.v` | I2C slave. Samples SCL / SDA with a system clock |
| `rtl/i2c_whoami_scl.v` | The same I2C slave without a system clock, running on SCL / SDA only |
| `rtl/spi_whoami.v` | SPI slave, clocked by SCK |
| `rtl/*_top.v` | Chip-level tops: power pins, power-on reset, SDA / MISO as `inout` |
| `rtl/sim/` | Simulation-only wrappers that let Verilator use the `*_top.v` modules |
| `tb/` | Icarus Verilog unit testbenches |

```sh
make lint   # lint with Verilator
make sim    # run the unit tests with Icarus Verilog
```
