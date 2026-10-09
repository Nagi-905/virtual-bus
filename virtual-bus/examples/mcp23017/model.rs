//! Transaction-level model of the MCP23017 (16-bit I/O expander).
//!
//! An example of writing an IC model with virtual-bus's building blocks ([`I2cSlave`]).
//! Included with `mod` from both the example (`main.rs`) and the test (`tests/mcp23017.rs`).
//!
//! Register layout for IOCON.BANK = 0. The first written byte is the register pointer;
//! after that it auto-increments through 0x00..=0x15 byte by byte (SEQOP = 0).
//!
//! Supported: IODIR / IPOL / GPPU / GPIO / OLAT. GPINTEN / DEFVAL / INTCON / IOCON only
//! hold their values; interrupts, BANK = 1 and SEQOP = 1 are not supported. INTF / INTCAP always read 0.
//!
//! Pins 0..=7 are GPA0..=GPA7, 8..=15 are GPB0..=GPB7.

// some constants and functions are unused depending on who includes this file
#![allow(dead_code)]

use virtual_bus::{Direction, I2cSlave, Nack};

pub const IODIRA: u8 = 0x00;
pub const IODIRB: u8 = 0x01;
pub const IPOLA: u8 = 0x02;
pub const IPOLB: u8 = 0x03;
pub const GPINTENA: u8 = 0x04;
pub const GPINTENB: u8 = 0x05;
pub const DEFVALA: u8 = 0x06;
pub const DEFVALB: u8 = 0x07;
pub const INTCONA: u8 = 0x08;
pub const INTCONB: u8 = 0x09;
pub const IOCONA: u8 = 0x0A;
pub const IOCONB: u8 = 0x0B;
pub const GPPUA: u8 = 0x0C;
pub const GPPUB: u8 = 0x0D;
pub const INTFA: u8 = 0x0E;
pub const INTFB: u8 = 0x0F;
pub const INTCAPA: u8 = 0x10;
pub const INTCAPB: u8 = 0x11;
pub const GPIOA: u8 = 0x12;
pub const GPIOB: u8 = 0x13;
pub const OLATA: u8 = 0x14;
pub const OLATB: u8 = 0x15;

const NUM_REGS: usize = 0x16;

/// MCP23017 model
#[derive(Debug, Clone)]
pub struct Mcp23017 {
    regs: [u8; NUM_REGS],
    ptr: u8,
    expect_ptr: bool,
    /// Pins driven from outside (None = not driven)
    external: [Option<bool>; 16],
}

impl Default for Mcp23017 {
    fn default() -> Self {
        Self::new()
    }
}

impl Mcp23017 {
    /// The state right after power-on reset (all pins are inputs)
    pub fn new() -> Self {
        let mut regs = [0u8; NUM_REGS];
        regs[IODIRA as usize] = 0xFF;
        regs[IODIRB as usize] = 0xFF;
        Self {
            regs,
            ptr: 0,
            expect_ptr: false,
            external: [None; 16],
        }
    }

    /// Drives a pin from outside (for testing input pins)
    pub fn set_pin(&mut self, pin: u8, level: bool) {
        self.external[pin as usize] = Some(level);
    }

    /// Stops driving a pin from outside
    pub fn release_pin(&mut self, pin: u8) {
        self.external[pin as usize] = None;
    }

    /// The actual level of all pins (bit n = pin n).
    /// Output pins follow OLAT; input pins follow external drive > pull-up (1) > floating (0)
    pub fn pins(&self) -> u16 {
        u16::from(self.port_levels(0)) | (u16::from(self.port_levels(1)) << 8)
    }

    /// The level of one pin
    pub fn pin(&self, pin: u8) -> bool {
        self.pins() & (1 << pin) != 0
    }

    /// The raw value of a register (GPIO gives the same value as a read)
    pub fn register(&self, addr: u8) -> u8 {
        self.read_reg(addr)
    }

    fn port_levels(&self, port: usize) -> u8 {
        let iodir = self.regs[IODIRA as usize + port];
        let olat = self.regs[OLATA as usize + port];
        let gppu = self.regs[GPPUA as usize + port];
        let mut v = 0u8;
        for bit in 0..8 {
            let mask = 1u8 << bit;
            let level = if iodir & mask == 0 {
                olat & mask != 0
            } else {
                match self.external[port * 8 + bit] {
                    Some(l) => l,
                    None => gppu & mask != 0,
                }
            };
            if level {
                v |= mask;
            }
        }
        v
    }

    fn read_reg(&self, addr: u8) -> u8 {
        match addr {
            GPIOA | GPIOB => {
                let port = (addr - GPIOA) as usize;
                let iodir = self.regs[IODIRA as usize + port];
                let ipol = self.regs[IPOLA as usize + port];
                self.port_levels(port) ^ (ipol & iodir)
            }
            INTFA | INTFB | INTCAPA | INTCAPB => 0,
            a if (a as usize) < NUM_REGS => self.regs[a as usize],
            _ => 0,
        }
    }

    fn write_reg(&mut self, addr: u8, value: u8) {
        match addr {
            // writes to GPIO go to OLAT
            GPIOA | GPIOB => self.regs[(addr - GPIOA + OLATA) as usize] = value,
            INTFA | INTFB | INTCAPA | INTCAPB => {}
            // IOCONA and IOCONB are the same register
            IOCONA | IOCONB => {
                self.regs[IOCONA as usize] = value;
                self.regs[IOCONB as usize] = value;
            }
            a if (a as usize) < NUM_REGS => self.regs[a as usize] = value,
            _ => {}
        }
    }

    fn advance(&mut self) {
        self.ptr = if self.ptr as usize >= NUM_REGS - 1 {
            0
        } else {
            self.ptr + 1
        };
    }
}

impl I2cSlave for Mcp23017 {
    fn start(&mut self, dir: Direction) -> Result<(), Nack> {
        self.expect_ptr = dir == Direction::Write;
        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Nack> {
        for &b in data {
            if self.expect_ptr {
                self.ptr = b;
                self.expect_ptr = false;
            } else {
                self.write_reg(self.ptr, b);
                self.advance();
            }
        }
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8]) {
        for b in buf {
            *b = self.read_reg(self.ptr);
            self.advance();
        }
    }
}
