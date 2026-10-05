//! Pin-level SPI signal lines (SCK / MOSI / MISO and one CS per device) and simulated time.
//!
//! - Models clocked directly by SCK (such as `spi_whoami.v`) are evaluated in
//!   [`SpiPinModel::set_inputs`] whenever a pin changes
//! - Models running on a system clock (such as ICs that synchronize SCK to a system clock) return
//!   [`SpiPinModel::period_ps`] and get [`SpiPinModel::tick`] as time advances
//!
//! MISO is pulled up to 1 when nobody drives it. Two or more drivers at once are counted
//! as contention ([`SimSpiBus::contentions`]), and low wins.
//!
//! CS polarity is chosen per device with [`CsPolarity`] ([`SimSpiBus::add_device_with`]).
//! Models get a logical `cs_n` (false while selected); the bus converts it to and from the physical level.

use std::cell::RefCell;
use std::convert::Infallible;
use std::rc::Rc;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin};
use embedded_hal::spi::{ErrorKind, Mode, Phase, Polarity};

use super::bitbang::BitBangSpi;
use super::{CsPolarity, SpiSlave};
use crate::bus::SimDelay;

/// A model attached to the SPI signal lines
pub trait SpiPinModel {
    /// An input pin changed (may also be called when nothing changed).
    /// `cs_n` is logical: false while selected, whatever the physical CS polarity
    fn set_inputs(&mut self, cs_n: bool, sck: bool, mosi: bool);
    /// The system clock period (ps), if the model runs on a system clock
    fn period_ps(&self) -> Option<u64> {
        None
    }
    /// Advances by one rising edge of the system clock
    fn tick(&mut self) {}
    /// The MISO output. `None` is high impedance
    fn miso(&self) -> Option<bool>;
    /// The simulated time (ps) moved to `now_ps`. Called before any `set_inputs` / `tick`
    /// at that time, and once when the model is attached. For models that dump waveforms
    fn set_time_ps(&mut self, _now_ps: u64) {}
}

/// Models wrapped with [`crate::shared`] can be attached too (to poke at and observe them afterwards)
impl<T: SpiPinModel + ?Sized> SpiPinModel for Rc<RefCell<T>> {
    fn set_inputs(&mut self, cs_n: bool, sck: bool, mosi: bool) {
        self.borrow_mut().set_inputs(cs_n, sck, mosi)
    }
    fn period_ps(&self) -> Option<u64> {
        self.borrow().period_ps()
    }
    fn tick(&mut self) {
        self.borrow_mut().tick()
    }
    fn miso(&self) -> Option<bool> {
        self.borrow().miso()
    }
    fn set_time_ps(&mut self, now_ps: u64) {
        self.borrow_mut().set_time_ps(now_ps)
    }
}

struct Dev {
    model: Box<dyn SpiPinModel>,
    polarity: CsPolarity,
    /// Logical value. False while selected
    cs_n: bool,
    /// When the device was last deselected
    deselected_since: u64,
    period: Option<u64>,
    next: u64,
}

struct World {
    now: u64,
    sck: bool,
    mosi: bool,
    devs: Vec<Dev>,
    contentions: usize,
    in_contention: bool,
    min_cs_high: u64,
    cs_high_violations: usize,
}

impl World {
    fn propagate(&mut self) {
        let (sck, mosi) = (self.sck, self.mosi);
        for d in &mut self.devs {
            d.model.set_inputs(d.cs_n, sck, mosi);
        }
        self.check_contention();
    }

    fn check_contention(&mut self) {
        let drivers = self
            .devs
            .iter()
            .filter(|d| d.model.miso().is_some())
            .count();
        if drivers >= 2 {
            if !self.in_contention {
                self.contentions += 1;
                self.in_contention = true;
            }
        } else {
            self.in_contention = false;
        }
    }

    /// Pulled up to 1 when undriven; low wins on contention
    fn miso(&self) -> bool {
        self.devs.iter().filter_map(|d| d.model.miso()).all(|v| v)
    }

    /// Moves the time to `t` and tells the models if it changed
    fn set_now(&mut self, t: u64) {
        if t != self.now {
            self.now = t;
            for d in &mut self.devs {
                d.model.set_time_ps(t);
            }
        }
    }

    fn advance(&mut self, dt: u64) {
        let end = self.now + dt;
        while let Some(t) = self
            .devs
            .iter()
            .filter(|d| d.period.is_some())
            .map(|d| d.next)
            .min()
            .filter(|&t| t <= end)
        {
            self.set_now(t);
            for d in &mut self.devs {
                if let Some(p) = d.period.filter(|_| d.next == t) {
                    d.model.tick();
                    d.next += p;
                }
            }
            self.check_contention();
        }
        self.set_now(end);
    }
}

/// Bit-bang SPI master returned by [`SimSpiBus::master`]
pub type SimSpiMaster = BitBangSpi<SimSpiPin, SimSpiPin, SimMisoPin, SimDelay>;

/// SPI signal lines. Cloning gives another handle to the same lines
#[derive(Clone)]
pub struct SimSpiBus {
    world: Rc<RefCell<World>>,
}

impl Default for SimSpiBus {
    fn default() -> Self {
        Self::new()
    }
}

impl SimSpiBus {
    pub fn new() -> Self {
        Self {
            world: Rc::new(RefCell::new(World {
                now: 0,
                sck: false,
                mosi: false,
                devs: Vec::new(),
                contentions: 0,
                in_contention: false,
                min_cs_high: 0,
                cs_high_violations: 0,
            })),
        }
    }

    /// Attaches a model and returns its CS pin (selected when low)
    pub fn add_device(&self, model: impl SpiPinModel + 'static) -> SimCsPin {
        self.add_device_with(model, CsPolarity::ActiveLow)
    }

    /// Attaches a model with the given CS polarity. The CS pin starts at the deselected level
    pub fn add_device_with(
        &self,
        model: impl SpiPinModel + 'static,
        polarity: CsPolarity,
    ) -> SimCsPin {
        let mut w = self.world.borrow_mut();
        let period = model.period_ps();
        let now = w.now;
        let (sck, mosi) = (w.sck, w.mosi);
        let mut model = Box::new(model);
        model.set_time_ps(now);
        model.set_inputs(true, sck, mosi);
        w.devs.push(Dev {
            model,
            polarity,
            cs_n: true,
            deselected_since: now,
            period,
            next: now + period.unwrap_or(0),
        });
        SimCsPin {
            world: self.world.clone(),
            index: w.devs.len() - 1,
        }
    }

    pub fn sck_pin(&self) -> SimSpiPin {
        SimSpiPin {
            world: self.world.clone(),
            sck: true,
        }
    }

    pub fn mosi_pin(&self) -> SimSpiPin {
        SimSpiPin {
            world: self.world.clone(),
            sck: false,
        }
    }

    pub fn miso_pin(&self) -> SimMisoPin {
        SimMisoPin {
            world: self.world.clone(),
        }
    }

    pub fn delay(&self) -> SimDelay {
        let w = self.world.clone();
        SimDelay::new(Rc::new(move |ps| w.borrow_mut().advance(ps)))
    }

    /// A bit-bang SPI master on these lines
    pub fn master(&self, mode: Mode, freq_hz: u32) -> Result<SimSpiMaster, ErrorKind> {
        BitBangSpi::new(
            self.sck_pin(),
            self.mosi_pin(),
            self.miso_pin(),
            self.delay(),
            mode,
            freq_hz,
        )
    }

    /// Minimum CS deselect time (t_CSH; the high time for active-low CS). Selecting again before
    /// this time has passed since deselecting is counted as a violation ([`Self::cs_high_violations`]),
    /// and time is advanced by the missing amount before selecting
    pub fn set_min_cs_high_ns(&self, ns: u64) {
        self.world.borrow_mut().min_cs_high = ns * 1000;
    }

    pub fn cs_high_violations(&self) -> usize {
        self.world.borrow().cs_high_violations
    }

    /// Number of times two or more devices drove MISO at once
    pub fn contentions(&self) -> usize {
        self.world.borrow().contentions
    }

    pub fn miso(&self) -> bool {
        self.world.borrow().miso()
    }

    pub fn run_ns(&self, ns: u64) {
        self.world.borrow_mut().advance(ns * 1000);
    }

    pub fn now_ps(&self) -> u64 {
        self.world.borrow().now
    }

    pub fn now_ns(&self) -> u64 {
        self.now_ps() / 1000
    }
}

/// The master's SCK / MOSI (push-pull outputs)
pub struct SimSpiPin {
    world: Rc<RefCell<World>>,
    sck: bool,
}

impl ErrorType for SimSpiPin {
    type Error = Infallible;
}

impl SimSpiPin {
    fn drive(&mut self, level: bool) {
        let mut w = self.world.borrow_mut();
        if self.sck {
            w.sck = level;
        } else {
            w.mosi = level;
        }
        w.propagate();
    }
}

impl OutputPin for SimSpiPin {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.drive(false);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Infallible> {
        self.drive(true);
        Ok(())
    }
}

/// The master's MISO input
pub struct SimMisoPin {
    world: Rc<RefCell<World>>,
}

impl ErrorType for SimMisoPin {
    type Error = Infallible;
}

impl InputPin for SimMisoPin {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(self.world.borrow().miso())
    }
    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(!self.world.borrow().miso())
    }
}

/// The CS of one device. Which level selects it depends on the [`CsPolarity`] given when attaching
pub struct SimCsPin {
    world: Rc<RefCell<World>>,
    index: usize,
}

impl ErrorType for SimCsPin {
    type Error = Infallible;
}

impl SimCsPin {
    fn drive(&mut self, level: bool) {
        let mut w = self.world.borrow_mut();
        let d = &w.devs[self.index];
        let select = d.polarity.is_selected(level);
        if select && d.cs_n {
            let idle_for = w.now - d.deselected_since;
            if idle_for < w.min_cs_high {
                w.cs_high_violations += 1;
                let wait = w.min_cs_high - idle_for;
                w.advance(wait);
            }
            w.devs[self.index].cs_n = false;
        } else if !select && !d.cs_n {
            let now = w.now;
            let d = &mut w.devs[self.index];
            d.cs_n = true;
            d.deselected_since = now;
        }
        w.propagate();
    }
}

impl OutputPin for SimCsPin {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.drive(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Infallible> {
        self.drive(true);
        Ok(())
    }
}

/// Adapter that runs a transaction-level [`SpiSlave`] at pin level, clocked directly by SCK.
///
/// `mode` is the slave's SPI mode. If it does not match the master's mode, the bits shift
/// just like on real hardware (no error is reported)
pub struct PinLevelSpiSlave<S> {
    slave: S,
    mode: Mode,
    selected: bool,
    prev_sck: bool,
    out: u8,
    inb: u8,
    bits: u8,
    miso: bool,
}

impl<S: SpiSlave> PinLevelSpiSlave<S> {
    pub fn new(slave: S, mode: Mode) -> Self {
        Self {
            slave,
            mode,
            selected: false,
            prev_sck: mode.polarity == Polarity::IdleHigh,
            out: 0,
            inb: 0,
            bits: 0,
            miso: true,
        }
    }

    pub fn slave(&self) -> &S {
        &self.slave
    }

    fn capture(&mut self, mosi: bool) {
        self.inb = (self.inb << 1) | u8::from(mosi);
        self.bits += 1;
        if self.bits == 8 {
            self.slave.receive(self.inb);
            self.bits = 0;
        }
    }
}

impl<S: SpiSlave> SpiPinModel for PinLevelSpiSlave<S> {
    fn set_inputs(&mut self, cs_n: bool, sck: bool, mosi: bool) {
        let idle = self.mode.polarity == Polarity::IdleHigh;
        let cpha0 = self.mode.phase == Phase::CaptureOnFirstTransition;
        if cs_n {
            if self.selected {
                self.selected = false;
                self.slave.deselect();
            }
            self.prev_sck = sck;
            return;
        }
        if !self.selected {
            self.selected = true;
            self.bits = 0;
            self.slave.select();
            if cpha0 {
                self.out = self.slave.next_miso();
                self.miso = self.out & 0x80 != 0;
            }
        }
        if sck == self.prev_sck {
            return;
        }
        self.prev_sck = sck;
        let leading = sck != idle;
        match (cpha0, leading) {
            // CPHA = 0: capture on the leading edge, shift out the next bit on the trailing edge
            (true, true) => self.capture(mosi),
            (true, false) => {
                if self.bits == 0 {
                    self.out = self.slave.next_miso();
                    self.miso = self.out & 0x80 != 0;
                } else {
                    self.miso = self.out & (0x80 >> self.bits) != 0;
                }
            }
            // CPHA = 1: shift out on the leading edge, capture on the trailing edge
            (false, true) => {
                if self.bits == 0 {
                    self.out = self.slave.next_miso();
                }
                self.miso = self.out & (0x80 >> self.bits) != 0;
            }
            (false, false) => self.capture(mosi),
        }
    }

    fn miso(&self) -> Option<bool> {
        self.selected.then_some(self.miso)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::register::{RegisterMap, SpiFormat, SpiRegisterDevice};
    use embedded_hal::spi::SpiDevice;
    use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3, SpiBus};
    use embedded_hal_bus::spi::ExclusiveDevice;

    /// A mode 0 pin-level model with WHO_AM_I (0x0F = 0x33)
    fn whoami_pin() -> PinLevelSpiSlave<SpiRegisterDevice> {
        let map = RegisterMap::new().ro(0x0F, 0x33).rw(0x20, 0x00);
        PinLevelSpiSlave::new(map.spi(SpiFormat::read_bit7_inc_bit6()), MODE_0)
    }

    /// A broken device that keeps driving MISO regardless of CS
    struct AlwaysDrive;
    impl SpiPinModel for AlwaysDrive {
        fn set_inputs(&mut self, _: bool, _: bool, _: bool) {}
        fn miso(&self) -> Option<bool> {
            Some(false)
        }
    }

    #[test]
    fn idle_miso_is_pulled_up() {
        let bus = SimSpiBus::new();
        let _cs = bus.add_device(whoami_pin());
        let mut spi = bus.master(MODE_0, 1_000_000).unwrap();
        let mut b = [0u8; 2];
        spi.read(&mut b).unwrap();
        assert_eq!(b, [0xFF, 0xFF]);
    }

    #[test]
    fn pin_level_adapter_runs_register_map_in_all_modes() {
        for mode in [MODE_0, MODE_1, MODE_2, MODE_3] {
            let map = RegisterMap::new().ro(0x0F, 0x33).rw(0x20, 0);
            let bus = SimSpiBus::new();
            let cs = bus.add_device(PinLevelSpiSlave::new(
                map.spi(SpiFormat::read_bit7_inc_bit6()),
                mode,
            ));
            let spi = bus.master(mode, 1_000_000).unwrap();
            let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
            let mut b = [0x8F, 0];
            dev.transfer_in_place(&mut b).unwrap();
            assert_eq!(b[1], 0x33, "{mode:?}");
            dev.write(&[0x20, 0xA7]).unwrap();
            assert_eq!(map.get(0x20), 0xA7, "{mode:?}");
        }
    }

    #[test]
    fn mismatched_mode_garbles_data_without_error() {
        let bus = SimSpiBus::new();
        let cs = bus.add_device(whoami_pin());
        let spi = bus.master(MODE_1, 1_000_000).unwrap();
        let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
        let mut b = [0x8F, 0];
        dev.transfer_in_place(&mut b).unwrap();
        assert_ne!(b[1], 0x33);
    }

    #[test]
    fn miso_contention_is_counted() {
        let bus = SimSpiBus::new();
        let cs = bus.add_device(whoami_pin());
        let _rogue = bus.add_device(AlwaysDrive);
        assert_eq!(bus.contentions(), 0);
        let spi = bus.master(MODE_0, 1_000_000).unwrap();
        let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
        let mut b = [0x8F, 0];
        dev.transfer_in_place(&mut b).unwrap();
        assert_eq!(bus.contentions(), 1);
        assert_eq!(b[1], 0x00); // low wins
        dev.transfer_in_place(&mut [0x8F, 0]).unwrap();
        assert_eq!(bus.contentions(), 2);
    }

    #[test]
    fn min_cs_high_time_is_checked_and_inserted() {
        let bus = SimSpiBus::new();
        bus.set_min_cs_high_ns(200);
        let mut cs = bus.add_device(whoami_pin());
        bus.run_ns(1000);
        cs.set_low().unwrap();
        cs.set_high().unwrap();
        let t0 = bus.now_ns();
        cs.set_low().unwrap();
        assert_eq!(bus.cs_high_violations(), 1);
        assert_eq!(bus.now_ns() - t0, 200);
        cs.set_high().unwrap();
        bus.run_ns(300);
        cs.set_low().unwrap();
        assert_eq!(bus.cs_high_violations(), 1);
    }

    #[test]
    fn clocked_models_tick_while_time_advances() {
        struct Counter(Rc<RefCell<u32>>);
        impl SpiPinModel for Counter {
            fn set_inputs(&mut self, _: bool, _: bool, _: bool) {}
            fn period_ps(&self) -> Option<u64> {
                Some(20_000)
            }
            fn tick(&mut self) {
                *self.0.borrow_mut() += 1;
            }
            fn miso(&self) -> Option<bool> {
                None
            }
        }
        let n = Rc::new(RefCell::new(0));
        let bus = SimSpiBus::new();
        let _cs = bus.add_device(Counter(n.clone()));
        bus.run_ns(1000);
        assert_eq!(*n.borrow(), 50);
    }

    #[test]
    fn active_low_and_active_high_devices_share_one_bus() {
        use crate::bus::pin::InvertedPin;
        let low = RegisterMap::new().ro(0x0F, 0x33);
        let high = RegisterMap::new().ro(0x0F, 0x44);
        let bus = SimSpiBus::new();
        let cs_low = bus.add_device(PinLevelSpiSlave::new(
            low.spi(SpiFormat::read_bit7_inc_bit6()),
            MODE_0,
        ));
        let cs_high = bus.add_device_with(
            PinLevelSpiSlave::new(high.spi(SpiFormat::read_bit7_inc_bit6()), MODE_0),
            CsPolarity::ActiveHigh,
        );
        // ExclusiveDevice owns the bus with one device, so drive CS by hand
        let mut spi = bus.master(MODE_0, 1_000_000).unwrap();
        let mut cs_low = cs_low;
        let mut cs_high = InvertedPin::new(cs_high);
        cs_low.set_high().unwrap();
        cs_high.set_high().unwrap(); // logical high = physical low = not selected

        for (cs, expected) in [
            (&mut cs_low as &mut dyn OutputPin<Error = Infallible>, 0x33),
            (&mut cs_high, 0x44),
        ] {
            cs.set_low().unwrap();
            let mut b = [0x8F, 0x00];
            spi.transfer_in_place(&mut b).unwrap();
            cs.set_high().unwrap();
            assert_eq!(b[1], expected);
        }
        assert_eq!(bus.contentions(), 0);
    }

    #[test]
    fn min_cs_idle_time_applies_to_active_high_cs() {
        let bus = SimSpiBus::new();
        bus.set_min_cs_high_ns(200);
        let mut cs = bus.add_device_with(whoami_pin(), CsPolarity::ActiveHigh);
        bus.run_ns(1000);
        cs.set_high().unwrap(); // select
        cs.set_low().unwrap(); // deselect
        let t0 = bus.now_ns();
        cs.set_high().unwrap(); // select right away → violation, waits 200 ns
        assert_eq!(bus.cs_high_violations(), 1);
        assert_eq!(bus.now_ns() - t0, 200);
    }
}
