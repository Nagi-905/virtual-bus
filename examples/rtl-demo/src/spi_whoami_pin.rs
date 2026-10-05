//! A pin-level model ported by hand from `rtl/spi_whoami.v` to Rust.
//!
//! Used to run the same conformance tests as the RTL (`tests/spi_conformance.rs`) without Verilator.

use virtual_bus::bus::spi::sim::SpiPinModel;

/// A pin-level model that behaves like `spi_whoami.v` (clocked directly by SCK, modes 0 / 3)
#[derive(Debug, Default)]
pub struct SpiWhoAmIPin {
    cs_n: bool,
    sck: bool,
    bitcnt: u8,
    shreg: u8,
    cmd_done: bool,
    rd: bool,
    ms: bool,
    addr: u8,
    ctrl: u8,
    tx: u8,
}

impl SpiWhoAmIPin {
    pub const WHO_AM_I: u8 = 0x33;

    pub fn new() -> Self {
        Self {
            cs_n: true,
            ..Default::default()
        }
    }

    pub fn ctrl(&self) -> u8 {
        self.ctrl
    }

    fn rdata(&self) -> u8 {
        match self.addr {
            0x0F => Self::WHO_AM_I,
            0x20 => self.ctrl,
            _ => 0,
        }
    }
}

impl SpiPinModel for SpiWhoAmIPin {
    fn set_inputs(&mut self, cs_n: bool, sck: bool, mosi: bool) {
        let posedge = sck && !self.sck;
        let negedge = !sck && self.sck;
        self.sck = sck;
        self.cs_n = cs_n;
        if cs_n {
            // the asynchronous reset of always @(... or posedge cs_n)
            self.bitcnt = 0;
            self.shreg = 0;
            self.cmd_done = false;
            self.rd = false;
            self.ms = false;
            self.addr = 0;
            self.tx = 0;
            return;
        }
        if posedge {
            let rx = (self.shreg << 1) | u8::from(mosi);
            if self.bitcnt == 7 {
                if !self.cmd_done {
                    self.cmd_done = true;
                    self.rd = rx & 0x80 != 0;
                    self.ms = rx & 0x40 != 0;
                    self.addr = rx & 0x3F;
                } else {
                    if !self.rd && self.addr == 0x20 {
                        self.ctrl = rx;
                    }
                    if self.ms {
                        self.addr = (self.addr + 1) & 0x3F;
                    }
                }
            }
            self.shreg = rx & 0x7F;
            self.bitcnt = (self.bitcnt + 1) & 7;
        }
        if negedge {
            self.tx = if self.bitcnt == 0 {
                if self.cmd_done && self.rd {
                    self.rdata()
                } else {
                    0
                }
            } else {
                self.tx << 1
            };
        }
    }

    fn miso(&self) -> Option<bool> {
        (!self.cs_n).then_some(self.tx & 0x80 != 0)
    }
}
