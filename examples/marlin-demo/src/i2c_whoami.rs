//! The I2C WHO_AM_I slaves of `rtl_demo::i2c_whoami`, loaded with Marlin.
//!
//! Same RTL and same behavior; only the way the model is built and its ports are accessed differ.
//!
//! - [`MarlinWhoAmI`]: the `i2c_whoami.v` core, oversampling SCL / SDA with a system clock
//! - [`MarlinWhoAmITop`]: the chip top without output enable, in its simulation wrapper
//! - [`MarlinWhoAmIScl`]: `i2c_whoami_scl.v`, running on SCL / SDA only

use std::path::Path;

use marlin::verilog::prelude::*;
use virtual_bus::bus::i2c::sim::I2cPinModel;

use crate::{Wave, create};

#[verilog(src = "../verilog/rtl/i2c_whoami.v", name = "i2c_whoami")]
struct Core;

#[verilog(src = "../verilog/rtl/sim/i2c_whoami_sim.v", name = "i2c_whoami_sim")]
struct Top;

#[verilog(src = "../verilog/rtl/i2c_whoami_scl.v", name = "i2c_whoami_scl")]
struct SclOnly;

const CORE_SOURCES: &[&str] = &["i2c_whoami.v"];
const TOP_SOURCES: &[&str] = &["sim/i2c_whoami_sim.v", "i2c_whoami_top.v", "i2c_whoami.v"];
const SCL_SOURCES: &[&str] = &["i2c_whoami_scl.v"];

/// I2C WHO_AM_I slave (address 0x29, 0x0F = 0xA5, 0x10 = SCRATCH). 50 MHz system clock
pub struct MarlinWhoAmI {
    m: Core<'static>,
    wave: Wave,
}

impl MarlinWhoAmI {
    pub const ADDRESS: u8 = 0x29;
    pub const WHO_AM_I: u8 = 0xA5;
    pub const PERIOD_PS: u64 = 20_000;

    /// Creates the model and resets it (`rst_n` 1 → 0 → 1; the reset is asynchronous)
    pub fn new() -> Self {
        Self::build(None)
    }

    /// Like [`Self::new`], and writes a VCD of all the RTL's signals to `path`
    pub fn with_vcd(path: impl AsRef<Path>) -> Self {
        Self::build(Some(path.as_ref()))
    }

    fn build(vcd: Option<&Path>) -> Self {
        let (mut m, wave) = create::<Core>(CORE_SOURCES, vcd);
        m.scl = 1;
        m.sda_i = 1;
        for level in [1, 0, 1] {
            m.rst_n = level;
            m.eval();
        }
        Self { m, wave }
    }
}

impl Default for MarlinWhoAmI {
    fn default() -> Self {
        Self::new()
    }
}

impl I2cPinModel for MarlinWhoAmI {
    fn set_inputs(&mut self, scl: bool, sda: bool) {
        self.m.scl = u8::from(scl);
        self.m.sda_i = u8::from(sda);
        // a synchronous circuit: evaluating on the system clock is enough.
        // When tracing, evaluate now too so the inputs show up in the waveform when they change
        if self.wave.is_on() {
            self.m.eval();
            self.wave.evaluated();
        }
    }

    fn period_ps(&self) -> Option<u64> {
        Some(Self::PERIOD_PS)
    }

    fn tick(&mut self) {
        self.m.clk = 0;
        self.m.eval();
        self.wave.clock_low(Self::PERIOD_PS);
        self.m.clk = 1;
        self.m.eval();
        self.wave.evaluated();
    }

    fn sda_low(&self) -> bool {
        self.m.sda_low != 0
    }

    fn set_time_ps(&mut self, now_ps: u64) {
        self.wave.set_time(now_ps);
    }
}

/// WHO_AM_I slave as a chip top without output enable (SDA resolved inside the Verilog wrapper).
///
/// No reset pin; the Verilog creates a power-on reset from vdd ([`Self::set_vdd`])
pub struct MarlinWhoAmITop {
    m: Top<'static>,
    wave: Wave,
    ext_low: bool,
}

impl MarlinWhoAmITop {
    pub const ADDRESS: u8 = MarlinWhoAmI::ADDRESS;
    pub const WHO_AM_I: u8 = MarlinWhoAmI::WHO_AM_I;

    /// Creates the model with power on
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
        m.scl = 1;
        m.ext_sda_low = 0;
        // move vdd 1 → 0 so the power-off reset surely happens (staying at the initial 0 gives no edge)
        for level in [1, 0] {
            m.vdd = level;
            m.eval();
        }
        Self {
            m,
            wave,
            ext_low: false,
        }
    }

    /// Turns the power (vdd) on / off
    pub fn set_vdd(&mut self, on: bool) {
        self.m.vdd = u8::from(on);
        self.m.eval();
        self.wave.evaluated();
    }

    /// The SDA level resolved inside the wrapper
    pub fn sda_level(&self) -> bool {
        self.m.sda_level != 0
    }
}

impl Default for MarlinWhoAmITop {
    fn default() -> Self {
        Self::new()
    }
}

impl I2cPinModel for MarlinWhoAmITop {
    fn set_inputs(&mut self, scl: bool, external_sda: bool) {
        self.ext_low = !external_sda;
        self.m.scl = u8::from(scl);
        self.m.ext_sda_low = u8::from(self.ext_low);
        // resolve the lines inside the wrapper (combinational; not a clock edge)
        self.m.eval();
        self.wave.evaluated();
    }

    fn period_ps(&self) -> Option<u64> {
        Some(MarlinWhoAmI::PERIOD_PS)
    }

    fn tick(&mut self) {
        self.m.clk = 0;
        self.m.eval();
        self.wave.clock_low(MarlinWhoAmI::PERIOD_PS);
        self.m.clk = 1;
        self.m.eval();
        self.wave.evaluated();
    }

    fn sda_low(&self) -> bool {
        !self.sda_level() && !self.ext_low
    }

    fn wants_external_sda(&self) -> bool {
        true
    }

    fn set_time_ps(&mut self, now_ps: u64) {
        self.wave.set_time(now_ps);
    }
}

/// WHO_AM_I slave without a system clock, evaluated whenever a line changes
pub struct MarlinWhoAmIScl {
    m: SclOnly<'static>,
    wave: Wave,
}

impl MarlinWhoAmIScl {
    pub const ADDRESS: u8 = MarlinWhoAmI::ADDRESS;
    pub const WHO_AM_I: u8 = MarlinWhoAmI::WHO_AM_I;

    /// Creates the model and resets it (`rst_n` 1 → 0 → 1)
    pub fn new() -> Self {
        Self::build(None)
    }

    /// Like [`Self::new`], and writes a VCD of all the RTL's signals to `path`
    pub fn with_vcd(path: impl AsRef<Path>) -> Self {
        Self::build(Some(path.as_ref()))
    }

    fn build(vcd: Option<&Path>) -> Self {
        let (mut m, wave) = create::<SclOnly>(SCL_SOURCES, vcd);
        m.scl = 1;
        m.sda_i = 1;
        for level in [1, 0, 1] {
            m.rst_n = level;
            m.eval();
        }
        Self { m, wave }
    }
}

impl Default for MarlinWhoAmIScl {
    fn default() -> Self {
        Self::new()
    }
}

impl I2cPinModel for MarlinWhoAmIScl {
    fn set_inputs(&mut self, scl: bool, sda: bool) {
        self.m.scl = u8::from(scl);
        self.m.sda_i = u8::from(sda);
        self.m.eval();
        self.wave.evaluated();
    }

    fn sda_low(&self) -> bool {
        self.m.sda_low != 0
    }

    fn set_time_ps(&mut self, now_ps: u64) {
        self.wave.set_time(now_ps);
    }
}
