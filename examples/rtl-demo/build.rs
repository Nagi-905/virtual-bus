//! Models the Verilog in ../verilog/rtl with Verilator.
//! The ports are read from the RTL; list only the sources of each model.

use virtual_bus_build::{Model, Verilated};

fn main() {
    Verilated::new()
        .rtl_dir("../verilog/rtl")
        // I2C: a WHO_AM_I slave without a system clock, running on SCL / SDA only
        .model(
            Model::new("i2c_whoami", "i2c_whoami")
                .source("i2c_whoami.v")
                .flag("-Wall"),
        )
        // I2C: the same slave as a chip top (inout SDA, power-on reset from vdd),
        // inside a simulation-only wrapper that resolves SDA
        .model(
            Model::new("i2c_whoami_sim", "i2c_whoami_sim")
                .source("sim/i2c_whoami_sim.v")
                .source("i2c_whoami_top.v")
                .source("i2c_whoami.v")
                .flag("-Wall"),
        )
        // SPI: a timer / counter on a system clock, sampling SCK / CS_N / MOSI with it
        .model(
            Model::new("spi_counter", "spi_counter")
                .source("spi_counter.v")
                .flag("-Wall")
                // can write a VCD (`with_vcd`)
                .trace(),
        )
        .compile();
}
