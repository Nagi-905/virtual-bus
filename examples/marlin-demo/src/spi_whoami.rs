//! The SPI WHO_AM_I slaves of `rtl_demo::spi_whoami`, loaded with Marlin.
//! The RTL is clocked directly by SCK, so it is evaluated whenever a pin changes.
//!
//! - [`MarlinSpiWhoAmI`]: the `spi_whoami.v` core (separate `miso` / `miso_oe` ports)
//! - [`MarlinSpiWhoAmITop`]: the chip top without output enable, in its simulation wrapper

use std::path::Path;

use marlin::verilog::prelude::*;
use virtual_bus::bus::spi::sim::SpiPinModel;

use crate::{Wave, create};

#[verilog(src = "../verilog/rtl/spi_whoami.v", name = "spi_whoami")]
struct Core;

#[verilog(src = "../verilog/rtl/sim/spi_whoami_sim.v", name = "spi_whoami_sim")]
struct Top;

const CORE_SOURCES: &[&str] = &["spi_whoami.v"];
const TOP_SOURCES: &[&str] = &["sim/spi_whoami_sim.v", "spi_whoami_top.v", "spi_whoami.v"];

/// SPI WHO_AM_I slave (0x0F = 0x33, 0x20 = CTRL, modes 0 / 3)
pub struct MarlinSpiWhoAmI {
    m: Core<'static>,
    wave: Wave,
}

impl MarlinSpiWhoAmI {
    pub const WHO_AM_I: u8 = 0x33;

    /// Creates the model and pulses `rst_n` (with `cs_n` high)
    pub fn new() -> Self {
        Self::build(None)
    }

    /// Like [`Self::new`], and writes a VCD of all the RTL's signals to `path`
    pub fn with_vcd(path: impl AsRef<Path>) -> Self {
        Self::build(Some(path.as_ref()))
    }

    fn build(vcd: Option<&Path>) -> Self {
        let (mut m, wave) = create::<Core>(CORE_SOURCES, vcd);
        m.cs_n = 1;
        for level in [1, 0, 1] {
            m.rst_n = level;
            m.eval();
        }
        Self { m, wave }
    }
}

impl Default for MarlinSpiWhoAmI {
    fn default() -> Self {
        Self::new()
    }
}

impl SpiPinModel for MarlinSpiWhoAmI {
    fn set_inputs(&mut self, cs_n: bool, sck: bool, mosi: bool) {
        self.m.cs_n = u8::from(cs_n);
        self.m.sck = u8::from(sck);
        self.m.mosi = u8::from(mosi);
        self.m.eval();
        self.wave.evaluated();
    }

    fn miso(&self) -> Option<bool> {
        (self.m.miso_oe != 0).then_some(self.m.miso != 0)
    }

    fn set_time_ps(&mut self, now_ps: u64) {
        self.wave.set_time(now_ps);
    }
}

/// SPI WHO_AM_I slave as a chip top without output enable (MISO resolved inside the Verilog wrapper).
///
/// As with `rtl_demo`, a driven 1 and Hi-Z cannot be told apart, so [`SpiPinModel::miso`] returns
/// `Some(false)` only when low. No reset pin; the Verilog creates a power-on reset from vdd
pub struct MarlinSpiWhoAmITop {
    m: Top<'static>,
    wave: Wave,
}

impl MarlinSpiWhoAmITop {
    pub const WHO_AM_I: u8 = MarlinSpiWhoAmI::WHO_AM_I;

    /// Creates the model with power on (`cs_n` high)
    pub fn new() -> Self {
        let mut s = Self::new_unpowered();
        s.set_vdd(true);
        s
    }

    /// Creates the model with power off
    pub fn new_unpowered() -> Self {
        Self::build(None)
    }

    /// Creates the model with power on, and writes a VCD of all the RTL's signals to `path`
    pub fn with_vcd(path: impl AsRef<Path>) -> Self {
        let mut s = Self::build(Some(path.as_ref()));
        s.set_vdd(true);
        s
    }

    fn build(vcd: Option<&Path>) -> Self {
        let (mut m, wave) = create::<Top>(TOP_SOURCES, vcd);
        m.cs_n = 1;
        // move vdd 1 → 0 so the power-off reset surely happens (staying at the initial 0 gives no edge)
        for level in [1, 0] {
            m.vdd = level;
            m.eval();
        }
        Self { m, wave }
    }

    /// Turns the power (vdd) on / off
    pub fn set_vdd(&mut self, on: bool) {
        self.m.vdd = u8::from(on);
        self.m.eval();
        self.wave.evaluated();
    }

    /// The MISO level resolved inside the wrapper
    pub fn miso_level(&self) -> bool {
        self.m.miso_level != 0
    }
}

impl Default for MarlinSpiWhoAmITop {
    fn default() -> Self {
        Self::new()
    }
}

impl SpiPinModel for MarlinSpiWhoAmITop {
    fn set_inputs(&mut self, cs_n: bool, sck: bool, mosi: bool) {
        self.m.cs_n = u8::from(cs_n);
        self.m.sck = u8::from(sck);
        self.m.mosi = u8::from(mosi);
        self.m.eval();
        self.wave.evaluated();
    }

    fn miso(&self) -> Option<bool> {
        (!self.miso_level()).then_some(false)
    }

    fn set_time_ps(&mut self, now_ps: u64) {
        self.wave.set_time(now_ps);
    }
}
