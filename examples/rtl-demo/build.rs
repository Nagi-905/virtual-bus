//! Models the Verilog in ../verilog/rtl with Verilator.
//! To add a port, just add an input / output here.

use virtual_bus_build::{Model, Verilated};

fn main() {
    Verilated::new()
        .rtl_dir("../verilog/rtl")
        // I2C: the WHO_AM_I slave core (oversampled with a system clock)
        .model(
            Model::new("i2c_whoami", "i2c_whoami")
                .source("i2c_whoami.v")
                .flag("-Wall")
                .input("clk", 1)
                .input("rst_n", 1)
                .input("scl", 1)
                .input("sda_i", 1)
                .output("sda_low", 1),
        )
        // I2C: the chip top without output enable or reset pins (vdd + power-on reset),
        // wrapped in a simulation-only wrapper
        .model(
            Model::new("i2c_whoami_sim", "i2c_whoami_sim")
                .source("sim/i2c_whoami_sim.v")
                .source("i2c_whoami_top.v")
                .source("i2c_whoami.v")
                .flag("-Wall")
                .input("clk", 1)
                .input("vdd", 1)
                .input("scl", 1)
                .input("ext_sda_low", 1)
                .output("sda_level", 1),
        )
        // I2C: a WHO_AM_I slave without a system clock, running on SCL / SDA only
        .model(
            Model::new("i2c_whoami_scl", "i2c_whoami_scl")
                .source("i2c_whoami_scl.v")
                .flag("-Wall")
                .input("rst_n", 1)
                .input("scl", 1)
                .input("sda_i", 1)
                .output("sda_low", 1),
        )
        // SPI: the WHO_AM_I slave core (clocked directly by SCK)
        .model(
            Model::new("spi_whoami", "spi_whoami")
                .source("spi_whoami.v")
                .flag("-Wall")
                .input("rst_n", 1)
                .input("cs_n", 1)
                .input("sck", 1)
                .input("mosi", 1)
                .output("miso", 1)
                .output("miso_oe", 1),
        )
        // SPI: the chip top without output enable or reset pins, wrapped in a simulation-only wrapper
        .model(
            Model::new("spi_whoami_sim", "spi_whoami_sim")
                .source("sim/spi_whoami_sim.v")
                .source("spi_whoami_top.v")
                .source("spi_whoami.v")
                .flag("-Wall")
                .input("vdd", 1)
                .input("cs_n", 1)
                .input("sck", 1)
                .input("mosi", 1)
                .output("miso_level", 1),
        )
        .compile();
}
