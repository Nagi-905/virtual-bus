//! SPI.
//!
//! Transaction level (this file):
//!
//! - [`VirtualSpiDevice`]: presents one [`SpiSlave`] as a [`SpiDevice`].
//!   For ordinary drivers that leave CS to the bus
//! - [`virtual_spi_bus`]: returns `(VirtualSpiBus, VirtualCs)`. For drivers that take a [`SpiBus`] and
//!   a CS pin separately and drive CS themselves. Reads give 0xFF while CS is not asserted
//!
//! Pin level: [`bitbang::BitBangSpi`] drives the signal lines of [`sim::SimSpiBus`].

use std::cell::RefCell;
use std::collections::VecDeque;
use std::convert::Infallible;
use std::rc::Rc;

use embedded_hal::digital::{self, OutputPin};
use embedded_hal::spi::{self, ErrorKind, Mode, Operation, SpiBus, SpiDevice};

pub mod bitbang;
pub mod sim;

/// Transaction-level SPI slave model.
///
/// Real SPI is full duplex: the MISO of a byte must be decided before the MOSI of that byte
/// has been received. That is why one byte exchange is split into two steps,
/// [`next_miso`](Self::next_miso) → [`receive`](Self::receive).
/// With this split, the same model also runs behind the pin-level adapter
/// ([`crate::bus::spi::sim::PinLevelSpiSlave`]).
pub trait SpiSlave {
    /// Supported SPI modes. Using an unsupported mode gives [`SpiError::ModeFault`]
    fn supports_mode(&self, _mode: Mode) -> bool {
        true
    }
    /// CS was asserted
    fn select(&mut self);
    /// The MISO byte to shift out next
    fn next_miso(&mut self) -> u8;
    /// One byte was received
    fn receive(&mut self, mosi: u8);
    /// CS was deasserted
    fn deselect(&mut self);

    /// Exchanges one byte
    fn exchange(&mut self, mosi: u8) -> u8 {
        let miso = self.next_miso();
        self.receive(mosi);
        miso
    }
}

impl<T: SpiSlave + ?Sized> SpiSlave for Rc<RefCell<T>> {
    fn supports_mode(&self, mode: Mode) -> bool {
        self.borrow().supports_mode(mode)
    }
    fn select(&mut self) {
        self.borrow_mut().select()
    }
    fn next_miso(&mut self) -> u8 {
        self.borrow_mut().next_miso()
    }
    fn receive(&mut self, mosi: u8) {
        self.borrow_mut().receive(mosi)
    }
    fn deselect(&mut self) {
        self.borrow_mut().deselect()
    }
}

impl<T: SpiSlave + ?Sized> SpiSlave for Box<T> {
    fn supports_mode(&self, mode: Mode) -> bool {
        (**self).supports_mode(mode)
    }
    fn select(&mut self) {
        (**self).select()
    }
    fn next_miso(&mut self) -> u8 {
        (**self).next_miso()
    }
    fn receive(&mut self, mosi: u8) {
        (**self).receive(mosi)
    }
    fn deselect(&mut self) {
        (**self).deselect()
    }
}

/// Errors of the virtual SPI bus
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiError {
    /// An SPI mode the slave does not support
    ModeFault,
    /// An error injected with [`SpiFault::Error`]
    Injected(ErrorKind),
}

impl spi::Error for SpiError {
    fn kind(&self) -> ErrorKind {
        match self {
            SpiError::ModeFault => ErrorKind::ModeFault,
            SpiError::Injected(k) => *k,
        }
    }
}

/// A fault injected with [`VirtualSpiDevice::inject_fault`]. Affects only the next transaction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiFault {
    /// Return an error without starting the transaction
    Error(ErrorKind),
    /// MISO is stuck at a fixed value (the slave still receives MOSI)
    MisoStuck(u8),
    /// Invert the MISO of the `index`-th byte (0-based) with `mask`
    FlipBits { index: usize, mask: u8 },
}

/// The record of one transaction
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SpiRecord {
    pub mosi: Vec<u8>,
    pub miso: Vec<u8>,
    /// Total of the `Operation::DelayNs` operations
    pub delay_ns: u64,
}

/// Presents a [`SpiSlave`] as a [`SpiDevice`]
pub struct VirtualSpiDevice<S> {
    slave: S,
    mode: Mode,
    faults: VecDeque<SpiFault>,
    log: Vec<SpiRecord>,
}

impl<S: SpiSlave> VirtualSpiDevice<S> {
    /// `mode` is the SPI mode of the master (the driver side)
    pub fn new(slave: S, mode: Mode) -> Self {
        Self {
            slave,
            mode,
            faults: VecDeque::new(),
            log: Vec::new(),
        }
    }

    pub fn slave(&self) -> &S {
        &self.slave
    }

    pub fn slave_mut(&mut self) -> &mut S {
        &mut self.slave
    }

    pub fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    pub fn inject_fault(&mut self, fault: SpiFault) {
        self.faults.push_back(fault);
    }

    pub fn log(&self) -> &[SpiRecord] {
        &self.log
    }

    pub fn clear_log(&mut self) {
        self.log.clear();
    }
}

impl<S> spi::ErrorType for VirtualSpiDevice<S> {
    type Error = SpiError;
}

fn apply_fault(fault: Option<SpiFault>, index: usize, miso: u8) -> u8 {
    match fault {
        Some(SpiFault::MisoStuck(v)) => v,
        Some(SpiFault::FlipBits { index: i, mask }) if i == index => miso ^ mask,
        _ => miso,
    }
}

impl<S: SpiSlave> SpiDevice<u8> for VirtualSpiDevice<S> {
    fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), SpiError> {
        if !self.slave.supports_mode(self.mode) {
            return Err(SpiError::ModeFault);
        }
        let fault = self.faults.pop_front();
        if let Some(SpiFault::Error(kind)) = fault {
            return Err(SpiError::Injected(kind));
        }

        self.slave.select();
        let mut rec = SpiRecord::default();
        let slave = &mut self.slave;
        let mut xfer = |mosi: u8, rec: &mut SpiRecord| {
            let miso = apply_fault(fault, rec.mosi.len(), slave.exchange(mosi));
            rec.mosi.push(mosi);
            rec.miso.push(miso);
            miso
        };
        for op in ops.iter_mut() {
            match op {
                Operation::Read(buf) => {
                    for b in buf.iter_mut() {
                        *b = xfer(0x00, &mut rec);
                    }
                }
                Operation::Write(data) => {
                    for &b in data.iter() {
                        xfer(b, &mut rec);
                    }
                }
                Operation::Transfer(read, write) => {
                    for i in 0..read.len().max(write.len()) {
                        let miso = xfer(write.get(i).copied().unwrap_or(0x00), &mut rec);
                        if let Some(r) = read.get_mut(i) {
                            *r = miso;
                        }
                    }
                }
                Operation::TransferInPlace(buf) => {
                    for b in buf.iter_mut() {
                        *b = xfer(*b, &mut rec);
                    }
                }
                Operation::DelayNs(ns) => rec.delay_ns += u64::from(*ns),
            }
        }
        self.slave.deselect();
        self.log.push(rec);
        Ok(())
    }
}

/// CS polarity. Chosen when attaching a device to a bus
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CsPolarity {
    /// Selected when low (most ICs)
    #[default]
    ActiveLow,
    /// Selected when high. When passing CS to `ExclusiveDevice` or a driver,
    /// wrap it in [`crate::bus::pin::InvertedPin`]
    ActiveHigh,
}

impl CsPolarity {
    /// Whether this CS pin level (true = high) selects the device
    pub fn is_selected(self, level: bool) -> bool {
        match self {
            CsPolarity::ActiveLow => !level,
            CsPolarity::ActiveHigh => level,
        }
    }

    /// The level when not selected (true = high)
    pub fn idle_level(self) -> bool {
        self == CsPolarity::ActiveLow
    }
}

struct BusDevice {
    slave: Box<dyn SpiSlave>,
    polarity: CsPolarity,
    selected: bool,
}

struct BusInner {
    mode: Mode,
    devices: Vec<BusDevice>,
    contentions: usize,
}

/// Virtual SPI bus without CS ([`SpiBus`]). Cloning gives another handle to the same bus
#[derive(Clone)]
pub struct VirtualSpiBus {
    inner: Rc<RefCell<BusInner>>,
}

/// The CS pin of one device on a [`VirtualSpiBus`].
/// Which level selects it depends on the [`CsPolarity`] given when attaching
pub struct VirtualCs {
    inner: Rc<RefCell<BusInner>>,
    index: usize,
}

/// Creates a virtual SPI bus with a single device, and its CS (selected when low)
pub fn virtual_spi_bus(slave: impl SpiSlave + 'static, mode: Mode) -> (VirtualSpiBus, VirtualCs) {
    virtual_spi_bus_with(slave, mode, CsPolarity::ActiveLow)
}

/// [`virtual_spi_bus`] with a choice of CS polarity
pub fn virtual_spi_bus_with(
    slave: impl SpiSlave + 'static,
    mode: Mode,
    polarity: CsPolarity,
) -> (VirtualSpiBus, VirtualCs) {
    let bus = VirtualSpiBus::new(mode);
    let cs = bus.add_device_with(slave, polarity);
    (bus, cs)
}

impl VirtualSpiBus {
    pub fn new(mode: Mode) -> Self {
        Self {
            inner: Rc::new(RefCell::new(BusInner {
                mode,
                devices: Vec::new(),
                contentions: 0,
            })),
        }
    }

    /// Adds a device and returns its CS (selected when low)
    pub fn add_device(&self, slave: impl SpiSlave + 'static) -> VirtualCs {
        self.add_device_with(slave, CsPolarity::ActiveLow)
    }

    /// Adds a device with the given CS polarity
    pub fn add_device_with(
        &self,
        slave: impl SpiSlave + 'static,
        polarity: CsPolarity,
    ) -> VirtualCs {
        let mut inner = self.inner.borrow_mut();
        inner.devices.push(BusDevice {
            slave: Box::new(slave),
            polarity,
            selected: false,
        });
        VirtualCs {
            inner: self.inner.clone(),
            index: inner.devices.len() - 1,
        }
    }

    /// Number of bytes during which two or more devices were selected at once
    pub fn contentions(&self) -> usize {
        self.inner.borrow().contentions
    }

    fn exchange(&mut self, mosi: u8) -> Result<u8, SpiError> {
        let mut inner = self.inner.borrow_mut();
        let mode = inner.mode;
        let mut miso = 0xFFu8; // pulled up when nobody drives it
        let mut drivers = 0;
        for d in inner.devices.iter_mut().filter(|d| d.selected) {
            if !d.slave.supports_mode(mode) {
                return Err(SpiError::ModeFault);
            }
            miso &= d.slave.exchange(mosi);
            drivers += 1;
        }
        if drivers > 1 {
            inner.contentions += 1;
        }
        Ok(miso)
    }
}

impl spi::ErrorType for VirtualSpiBus {
    type Error = SpiError;
}

impl SpiBus<u8> for VirtualSpiBus {
    fn read(&mut self, words: &mut [u8]) -> Result<(), SpiError> {
        for w in words {
            *w = self.exchange(0x00)?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), SpiError> {
        for &w in words {
            self.exchange(w)?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), SpiError> {
        for i in 0..read.len().max(write.len()) {
            let miso = self.exchange(write.get(i).copied().unwrap_or(0x00))?;
            if let Some(r) = read.get_mut(i) {
                *r = miso;
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), SpiError> {
        for w in words {
            *w = self.exchange(*w)?;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), SpiError> {
        Ok(())
    }
}

impl digital::ErrorType for VirtualCs {
    type Error = Infallible;
}

impl VirtualCs {
    fn drive(&mut self, level: bool) {
        let mut inner = self.inner.borrow_mut();
        let d = &mut inner.devices[self.index];
        let selected = d.polarity.is_selected(level);
        if selected && !d.selected {
            d.selected = true;
            d.slave.select();
        } else if !selected && d.selected {
            d.selected = false;
            d.slave.deselect();
        }
    }
}

impl OutputPin for VirtualCs {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.drive(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Infallible> {
        self.drive(true);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shared;
    use embedded_hal::spi::{MODE_0, MODE_1, MODE_3};

    /// An echo that returns the received byte + 1 next
    #[derive(Default)]
    struct Echo {
        next: u8,
        selects: usize,
        deselects: usize,
        mode0_only: bool,
    }

    impl SpiSlave for Echo {
        fn supports_mode(&self, mode: Mode) -> bool {
            !self.mode0_only || mode == MODE_0
        }
        fn select(&mut self) {
            self.selects += 1;
            self.next = 0xA0;
        }
        fn next_miso(&mut self) -> u8 {
            self.next
        }
        fn receive(&mut self, mosi: u8) {
            self.next = mosi.wrapping_add(1);
        }
        fn deselect(&mut self) {
            self.deselects += 1;
        }
    }

    #[test]
    fn device_operations_follow_embedded_hal_semantics() {
        let mut dev = VirtualSpiDevice::new(Echo::default(), MODE_0);
        let mut r = [0u8; 3];
        let mut inplace = [0x10, 0x20];
        dev.transaction(&mut [
            Operation::Write(&[0x01]),
            Operation::Transfer(&mut r, &[0x05]), // the write side is shorter: the rest is 0x00
            Operation::DelayNs(500),
            Operation::TransferInPlace(&mut inplace),
        ])
        .unwrap();
        assert_eq!(r, [0x02, 0x06, 0x01]);
        assert_eq!(inplace, [0x01, 0x11]);
        let rec = &dev.log()[0];
        assert_eq!(rec.mosi, [0x01, 0x05, 0x00, 0x00, 0x10, 0x20]);
        assert_eq!(rec.delay_ns, 500);
        assert_eq!(dev.slave().selects, 1);
        assert_eq!(dev.slave().deselects, 1);
    }

    #[test]
    fn transfer_with_longer_write_discards_extra_reads() {
        let mut dev = VirtualSpiDevice::new(Echo::default(), MODE_0);
        let mut r = [0u8; 1];
        dev.transfer(&mut r, &[0x01, 0x02, 0x03]).unwrap();
        assert_eq!(r, [0xA0]);
        assert_eq!(dev.log()[0].mosi.len(), 3);
    }

    #[test]
    fn unsupported_mode_is_mode_fault() {
        let mut dev = VirtualSpiDevice::new(
            Echo {
                mode0_only: true,
                ..Default::default()
            },
            MODE_3,
        );
        assert_eq!(dev.write(&[0]), Err(SpiError::ModeFault));
        assert_eq!(spi::Error::kind(&SpiError::ModeFault), ErrorKind::ModeFault);
        dev.set_mode(MODE_0);
        dev.write(&[0]).unwrap();
    }

    #[test]
    fn injected_faults() {
        let mut dev = VirtualSpiDevice::new(Echo::default(), MODE_0);
        dev.inject_fault(SpiFault::Error(ErrorKind::Overrun));
        dev.inject_fault(SpiFault::MisoStuck(0xFF));
        dev.inject_fault(SpiFault::FlipBits {
            index: 1,
            mask: 0x01,
        });
        assert_eq!(dev.write(&[0]), Err(SpiError::Injected(ErrorKind::Overrun)));
        let mut b = [0x00, 0x00];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(b, [0xFF, 0xFF]);
        let mut b = [0x00, 0x00];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(b, [0xA0, 0x00]); // the 2nd byte is 0x01 ^ 0x01
        let mut b = [0x00, 0x00];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(b, [0xA0, 0x01]);
    }

    #[test]
    fn bus_reads_ff_without_cs_and_tracks_selection() {
        let echo = shared(Echo::default());
        let (mut bus, mut cs) = virtual_spi_bus(echo.clone(), MODE_0);
        let mut b = [0u8; 2];
        bus.read(&mut b).unwrap();
        assert_eq!(b, [0xFF, 0xFF]);
        cs.set_low().unwrap();
        cs.set_low().unwrap(); // asserting twice selects only once
        let mut b = [0x00, 0x00];
        bus.transfer_in_place(&mut b).unwrap();
        cs.set_high().unwrap();
        assert_eq!(b, [0xA0, 0x01]);
        assert_eq!(echo.borrow().selects, 1);
        assert_eq!(echo.borrow().deselects, 1);
    }

    #[test]
    fn two_selected_devices_are_counted_as_contention() {
        let bus = VirtualSpiBus::new(MODE_0);
        let mut cs1 = bus.add_device(Echo::default());
        let mut cs2 = bus.add_device(Echo::default());
        let mut bus2 = bus.clone();
        cs1.set_low().unwrap();
        cs2.set_low().unwrap();
        bus2.write(&[1, 2, 3]).unwrap();
        assert_eq!(bus.contentions(), 3);
    }

    #[test]
    fn bus_mode_fault() {
        let (mut bus, mut cs) = virtual_spi_bus(
            Echo {
                mode0_only: true,
                ..Default::default()
            },
            MODE_1,
        );
        cs.set_low().unwrap();
        assert_eq!(bus.write(&[0]), Err(SpiError::ModeFault));
    }

    #[test]
    fn active_high_cs_selects_on_high_level() {
        let echo = shared(Echo::default());
        let (mut bus, mut cs) = virtual_spi_bus_with(echo.clone(), MODE_0, CsPolarity::ActiveHigh);
        let mut b = [0x00, 0x00];
        cs.set_low().unwrap(); // active high, so low means not selected
        bus.transfer_in_place(&mut b).unwrap();
        assert_eq!(b, [0xFF, 0xFF]);
        assert_eq!(echo.borrow().selects, 0);

        cs.set_high().unwrap();
        let mut b = [0x00, 0x00];
        bus.transfer_in_place(&mut b).unwrap();
        assert_eq!(b, [0xA0, 0x01]);
        cs.set_low().unwrap();
        assert_eq!((echo.borrow().selects, echo.borrow().deselects), (1, 1));
    }

    #[test]
    fn exclusive_device_needs_inverted_pin_for_active_high_cs() {
        use crate::bus::pin::InvertedPin;
        use crate::devices::register::{RegisterMap, SpiFormat};
        use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

        let map = RegisterMap::new().ro(0x0F, 0x33);
        // ExclusiveDevice selects by driving CS low, so on its own it cannot select this device
        let (bus, cs) = virtual_spi_bus_with(
            map.spi(SpiFormat::read_bit7_inc_bit6()),
            MODE_0,
            CsPolarity::ActiveHigh,
        );
        let mut dev = ExclusiveDevice::new(bus, cs, NoDelay).unwrap();
        let mut b = [0x8F, 0x00];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(b[1], 0xFF);

        // wrapping CS in InvertedPin makes it work (like an inverter on real hardware)
        let (bus, cs) = virtual_spi_bus_with(
            map.spi(SpiFormat::read_bit7_inc_bit6()),
            MODE_0,
            CsPolarity::ActiveHigh,
        );
        let mut dev = ExclusiveDevice::new(bus, InvertedPin::new(cs), NoDelay).unwrap();
        let mut b = [0x8F, 0x00];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(b[1], 0x33);
    }
}
