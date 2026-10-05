//! A bus-independent register map.
//!
//! A building block for devices where "the first byte selects the address, and the rest reads or writes".
//! Define the registers in a [`RegisterMap`], then attach a front end per bus with
//! [`RegisterMap::spi`] / [`RegisterMap::i2c`].
//! Both can be attached to the same map (clones refer to the same map).

use std::cell::RefCell;
use std::rc::Rc;

use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, Mode};

use crate::bus::spi::SpiSlave;
use crate::{Direction, I2cSlave, Nack};

/// Access type of a register
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// Readable and writable from the bus
    ReadWrite,
    /// Writes from the bus are ignored
    ReadOnly,
}

struct Inner {
    values: [u8; 256],
    access: [Access; 256],
    writes: Vec<(u8, u8)>,
}

/// A 256-byte register map. Undefined addresses are read-only and read as 0
#[derive(Clone)]
pub struct RegisterMap {
    inner: Rc<RefCell<Inner>>,
}

impl Default for RegisterMap {
    fn default() -> Self {
        Self::new()
    }
}

impl RegisterMap {
    pub fn new() -> Self {
        Self {
            inner: Rc::new(RefCell::new(Inner {
                values: [0; 256],
                access: [Access::ReadOnly; 256],
                writes: Vec::new(),
            })),
        }
    }

    /// Defines a read-write register (builder)
    pub fn rw(self, addr: u8, init: u8) -> Self {
        self.define(addr, Access::ReadWrite, init);
        self
    }

    /// Defines a read-only register (builder)
    pub fn ro(self, addr: u8, value: u8) -> Self {
        self.define(addr, Access::ReadOnly, value);
        self
    }

    pub fn define(&self, addr: u8, access: Access, value: u8) {
        let mut i = self.inner.borrow_mut();
        i.access[addr as usize] = access;
        i.values[addr as usize] = value;
    }

    /// Reads a value (without going through the bus)
    pub fn get(&self, addr: u8) -> u8 {
        self.inner.borrow().values[addr as usize]
    }

    /// Injects a value (works on read-only registers too; not recorded in the write log)
    pub fn set(&self, addr: u8, value: u8) {
        self.inner.borrow_mut().values[addr as usize] = value;
    }

    pub fn access(&self, addr: u8) -> Access {
        self.inner.borrow().access[addr as usize]
    }

    /// A read from the bus
    pub fn bus_read(&self, addr: u8) -> u8 {
        self.get(addr)
    }

    /// A write from the bus. Ignored for read-only registers.
    /// Recorded in the write log either way
    pub fn bus_write(&self, addr: u8, value: u8) {
        let mut i = self.inner.borrow_mut();
        i.writes.push((addr, value));
        if i.access[addr as usize] == Access::ReadWrite {
            i.values[addr as usize] = value;
        }
    }

    /// The log of writes from the bus (address, value)
    pub fn writes(&self) -> Vec<(u8, u8)> {
        self.inner.borrow().writes.clone()
    }

    pub fn clear_writes(&self) {
        self.inner.borrow_mut().writes.clear();
    }

    /// Attaches an SPI front end
    pub fn spi(&self, format: SpiFormat) -> SpiRegisterDevice {
        SpiRegisterDevice {
            map: self.clone(),
            format,
            state: SpiState::Idle,
        }
    }

    /// Attaches an I2C front end
    pub fn i2c(&self, format: I2cFormat) -> I2cRegisterDevice {
        I2cRegisterDevice {
            map: self.clone(),
            format,
            ptr: 0,
            inc: true,
            expect_ptr: false,
        }
    }
}

/// Address auto-increment
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoIncrement {
    /// Always increment
    Always,
    /// Never increment (read or write the same register repeatedly)
    Never,
    /// Increment when this bit is set in the first byte
    Bit(u8),
}

impl AutoIncrement {
    fn enabled(self, first: u8) -> bool {
        match self {
            AutoIncrement::Always => true,
            AutoIncrement::Never => false,
            AutoIncrement::Bit(mask) => first & mask != 0,
        }
    }
}

/// How the first SPI byte is interpreted
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiFormat {
    /// The bit that encodes R/W
    pub rw_bit: u8,
    /// Read when `rw_bit` is set (if false, write when it is set)
    pub read_when_set: bool,
    /// The address bits
    pub addr_mask: u8,
    pub auto_increment: AutoIncrement,
    /// Supported SPI modes (modes 0..=3)
    pub modes: [bool; 4],
}

impl Default for SpiFormat {
    fn default() -> Self {
        Self::new()
    }
}

fn mode_index(mode: Mode) -> usize {
    match mode {
        m if m == MODE_0 => 0,
        m if m == MODE_1 => 1,
        m if m == MODE_2 => 2,
        _ => 3,
    }
}

impl SpiFormat {
    /// Read when bit7 = 1, address in bits 6..0, always auto-increment, all modes
    pub fn new() -> Self {
        Self {
            rw_bit: 0x80,
            read_when_set: true,
            addr_mask: 0x7F,
            auto_increment: AutoIncrement::Always,
            modes: [true; 4],
        }
    }

    /// Read when bit7 = 1, auto-increment when bit6 = 1, address in bits 5..0. Modes 0 / 3
    pub fn read_bit7_inc_bit6() -> Self {
        Self {
            rw_bit: 0x80,
            read_when_set: true,
            addr_mask: 0x3F,
            auto_increment: AutoIncrement::Bit(0x40),
            modes: [true, false, false, true],
        }
    }

    /// Write when `rw_bit` is set
    pub fn write_when_set(mut self) -> Self {
        self.read_when_set = false;
        self
    }

    pub fn auto_increment(mut self, ai: AutoIncrement) -> Self {
        self.auto_increment = ai;
        self
    }

    pub fn addr_mask(mut self, mask: u8) -> Self {
        self.addr_mask = mask;
        self
    }

    /// Sets the supported SPI modes
    pub fn modes(mut self, modes: &[Mode]) -> Self {
        self.modes = [false; 4];
        for &m in modes {
            self.modes[mode_index(m)] = true;
        }
        self
    }

    pub fn supports(&self, mode: Mode) -> bool {
        self.modes[mode_index(mode)]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SpiState {
    Idle,
    Command,
    Data { addr: u8, read: bool, inc: bool },
}

/// SPI front end of a [`RegisterMap`]
pub struct SpiRegisterDevice {
    map: RegisterMap,
    format: SpiFormat,
    state: SpiState,
}

impl SpiRegisterDevice {
    pub fn map(&self) -> &RegisterMap {
        &self.map
    }

    fn next_addr(&self, addr: u8) -> u8 {
        let m = self.format.addr_mask;
        (addr.wrapping_add(1) & m) | (addr & !m)
    }
}

impl SpiSlave for SpiRegisterDevice {
    fn supports_mode(&self, mode: Mode) -> bool {
        self.format.supports(mode)
    }

    fn select(&mut self) {
        self.state = SpiState::Command;
    }

    fn next_miso(&mut self) -> u8 {
        match self.state {
            SpiState::Data {
                addr, read: true, ..
            } => self.map.bus_read(addr),
            _ => 0x00,
        }
    }

    fn receive(&mut self, mosi: u8) {
        self.state = match self.state {
            SpiState::Idle => SpiState::Idle,
            SpiState::Command => {
                let f = &self.format;
                SpiState::Data {
                    addr: mosi & f.addr_mask,
                    read: (mosi & f.rw_bit != 0) == f.read_when_set,
                    inc: f.auto_increment.enabled(mosi),
                }
            }
            SpiState::Data { addr, read, inc } => {
                if !read {
                    self.map.bus_write(addr, mosi);
                }
                let addr = if inc { self.next_addr(addr) } else { addr };
                SpiState::Data { addr, read, inc }
            }
        };
    }

    fn deselect(&mut self) {
        self.state = SpiState::Idle;
    }
}

/// How the I2C sub-address (first byte) is interpreted
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cFormat {
    pub addr_mask: u8,
    pub auto_increment: AutoIncrement,
}

impl Default for I2cFormat {
    fn default() -> Self {
        Self::new()
    }
}

impl I2cFormat {
    /// 8-bit sub-address, always auto-increment
    pub fn new() -> Self {
        Self {
            addr_mask: 0xFF,
            auto_increment: AutoIncrement::Always,
        }
    }

    /// Auto-increment when bit7 of the sub-address = 1, address in bits 6..0
    pub fn inc_bit7() -> Self {
        Self {
            addr_mask: 0x7F,
            auto_increment: AutoIncrement::Bit(0x80),
        }
    }

    pub fn auto_increment(mut self, ai: AutoIncrement) -> Self {
        self.auto_increment = ai;
        self
    }
}

/// I2C front end of a [`RegisterMap`]. The address is chosen with [`crate::VirtualI2cBus::attach`]
pub struct I2cRegisterDevice {
    map: RegisterMap,
    format: I2cFormat,
    ptr: u8,
    inc: bool,
    expect_ptr: bool,
}

impl I2cRegisterDevice {
    pub fn map(&self) -> &RegisterMap {
        &self.map
    }

    fn advance(&mut self) {
        if self.inc {
            let m = self.format.addr_mask;
            self.ptr = (self.ptr.wrapping_add(1) & m) | (self.ptr & !m);
        }
    }
}

impl I2cSlave for I2cRegisterDevice {
    fn start(&mut self, dir: Direction) {
        self.expect_ptr = dir == Direction::Write;
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Nack> {
        for &b in data {
            if self.expect_ptr {
                self.expect_ptr = false;
                self.ptr = b & self.format.addr_mask;
                self.inc = self.format.auto_increment.enabled(b);
            } else {
                self.map.bus_write(self.ptr, b);
                self.advance();
            }
        }
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8]) {
        for b in buf {
            *b = self.map.bus_read(self.ptr);
            self.advance();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VirtualI2cBus;
    use crate::bus::spi::{SpiError, VirtualSpiDevice};
    use embedded_hal::i2c::I2c;
    use embedded_hal::spi::SpiDevice;

    fn map() -> RegisterMap {
        RegisterMap::new()
            .ro(0x0F, 0x33)
            .rw(0x20, 0x07)
            .rw(0x21, 0x00)
    }

    #[test]
    fn read_bit7_inc_bit6_spi_frame() {
        let m = map();
        let mut dev = VirtualSpiDevice::new(m.spi(SpiFormat::read_bit7_inc_bit6()), MODE_0);
        let mut b = [0x8F, 0];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(b[1], 0x33);
        // write with MS: goes into 0x20, 0x21 in turn
        dev.write(&[0x60, 0x57, 0x99]).unwrap();
        assert_eq!((m.get(0x20), m.get(0x21)), (0x57, 0x99));
        // read without MS: repeats the same register
        let mut b = [0xA0, 0, 0];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(&b[1..], [0x57, 0x57]);
        // read with MS
        let mut b = [0xE0, 0, 0];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(&b[1..], [0x57, 0x99]);
    }

    #[test]
    fn read_only_and_undefined_registers_ignore_writes_but_are_logged() {
        let m = map();
        let mut dev = VirtualSpiDevice::new(m.spi(SpiFormat::read_bit7_inc_bit6()), MODE_0);
        dev.write(&[0x0F, 0x00]).unwrap();
        dev.write(&[0x30, 0x12]).unwrap();
        assert_eq!(m.get(0x0F), 0x33);
        assert_eq!(m.get(0x30), 0x00);
        assert_eq!(m.writes(), [(0x0F, 0x00), (0x30, 0x12)]);
        m.set(0x0F, 0x44); // injection works on read-only registers too
        let mut b = [0x8F, 0];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(b[1], 0x44);
    }

    #[test]
    fn read_bit7_inc_bit6_rejects_mode_1() {
        let mut dev = VirtualSpiDevice::new(
            map().spi(SpiFormat::read_bit7_inc_bit6()),
            embedded_hal::spi::MODE_1,
        );
        assert_eq!(dev.write(&[0x8F, 0]), Err(SpiError::ModeFault));
    }

    #[test]
    fn write_when_set_format() {
        // a format where bit7 = 1 means write, with no address increment
        let m = RegisterMap::new().rw(0x00, 0).rw(0x01, 0);
        let fmt = SpiFormat::new()
            .write_when_set()
            .auto_increment(AutoIncrement::Never);
        let mut dev = VirtualSpiDevice::new(m.spi(fmt), MODE_0);
        dev.write(&[0x80, 0xF8]).unwrap();
        let mut b = [0x00, 0];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(b[1], 0xF8);
    }

    #[test]
    fn i2c_front_end_and_shared_map() {
        let m = map();
        let mut bus = VirtualI2cBus::new();
        bus.attach(0x18, m.i2c(I2cFormat::inc_bit7())).unwrap();
        let mut spi = VirtualSpiDevice::new(m.spi(SpiFormat::read_bit7_inc_bit6()), MODE_0);

        // write over I2C, read over SPI
        bus.write(0x18, &[0xA0, 0x11, 0x22]).unwrap(); // bit7 = auto-increment
        let mut b = [0xE0, 0, 0];
        spi.transfer_in_place(&mut b).unwrap();
        assert_eq!(&b[1..], [0x11, 0x22]);

        // without bit7, the same register
        let mut r = [0u8; 2];
        bus.write_read(0x18, &[0x20], &mut r).unwrap();
        assert_eq!(r, [0x11, 0x11]);
        bus.write_read(0x18, &[0x0F], &mut r[..1]).unwrap();
        assert_eq!(r[0], 0x33);
    }
}
