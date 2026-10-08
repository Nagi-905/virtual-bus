//! Pin-level SPI signal lines (SCK / MOSI / MISO and one CS per device) and simulated time.
//!
//! - Models clocked directly by SCK are evaluated in [`SpiPinModel::set_inputs`] whenever a pin changes
//! - Models running on a system clock (such as ICs that synchronize SCK to a system clock, like
//!   `spi_counter.v` in the examples) return [`SpiPinModel::period_ps`] and get [`SpiPinModel::tick`]
//!   as time advances
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
    /// When the device was last deselected. `None` until it has been selected once
    deselected_since: Option<u64>,
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
    /// t_CSH set by the user (ps). Shorter idle times are violations
    min_cs_high: u64,
    /// CS high time the bus always keeps between frames (ps), like a real master's gap.
    /// Set by `master` to one SCK period, or by `set_cs_high_ns`
    cs_high: u64,
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
                cs_high: 0,
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
            deselected_since: None,
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

    /// A bit-bang SPI master on these lines.
    ///
    /// Also sets the CS high time between frames to one SCK period of this master
    /// ([`Self::set_cs_high_ns`]), so frames sent through `ExclusiveDevice` stay apart
    pub fn master(&self, mode: Mode, freq_hz: u32) -> Result<SimSpiMaster, ErrorKind> {
        let spi = BitBangSpi::new(
            self.sck_pin(),
            self.mosi_pin(),
            self.miso_pin(),
            self.delay(),
            mode,
            freq_hz,
        )?;
        // the same rounding as the master's half period
        let half_ns = u64::from((1_000_000_000 / freq_hz / 2).max(1));
        self.set_cs_high_ns(2 * half_ns);
        Ok(spi)
    }

    /// The time the bus keeps CS deselected between frames, as a real SPI master leaves a gap.
    /// Selecting a device again sooner advances time by the rest first; this is not a violation.
    ///
    /// Pin writes take no simulated time, so without a gap `ExclusiveDevice` deselects and selects
    /// at the same moment: a slave that synchronizes CS to a system clock never sees the frame end,
    /// and the VCD shows CS staying low. [`Self::master`] sets this to one SCK period. Call it
    /// yourself when you build a master from [`Self::sck_pin`] and the other pins, or to model
    /// a master with a longer gap. The last call wins; it is 0 until then
    pub fn set_cs_high_ns(&self, ns: u64) {
        self.world.borrow_mut().cs_high = ns * 1000;
    }

    /// Minimum CS deselect time (t_CSH; the high time for active-low CS). Selecting again before
    /// this time has passed since deselecting is counted as a violation ([`Self::cs_high_violations`]),
    /// and time is advanced by the missing amount before selecting.
    ///
    /// This is the slave's requirement; [`Self::set_cs_high_ns`] is the gap the master leaves
    /// anyway (one SCK period with [`Self::master`]), and waiting for that is not a violation.
    ///
    /// Not needed for frames to work. Use it to check a driver against a datasheet t_CSH longer
    /// than one SCK period (for example a flash that needs tens of µs between commands): set the
    /// datasheet value, run the driver, then assert that [`Self::cs_high_violations`] is 0.
    /// A t_CSH shorter than one SCK period is always met by the bus's own wait
    ///
    /// ```
    /// use embedded_hal::spi::{MODE_0, SpiDevice};
    /// use embedded_hal_bus::spi::ExclusiveDevice;
    /// use virtual_bus::bus::spi::sim::{PinLevelSpiSlave, SimSpiBus};
    /// use virtual_bus::devices::register::{RegisterMap, SpiFormat};
    ///
    /// let regs = RegisterMap::new().rw(0x20, 0);
    /// let bus = SimSpiBus::new();
    /// bus.set_min_cs_high_ns(50_000); // the datasheet asks for 50 µs between frames
    /// let cs = bus.add_device(PinLevelSpiSlave::new(regs.spi(SpiFormat::read_bit7_inc_bit6()), MODE_0));
    /// let spi = bus.master(MODE_0, 1_000_000).unwrap();
    /// let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    ///
    /// dev.write(&[0x20, 0x01]).unwrap();
    /// dev.write(&[0x20, 0x02]).unwrap(); // right away: only 1 µs of CS high
    /// assert_eq!(bus.cs_high_violations(), 1); // this driver code does not wait long enough
    /// ```
    pub fn set_min_cs_high_ns(&self, ns: u64) {
        self.world.borrow_mut().min_cs_high = ns * 1000;
    }

    /// Number of times a device was selected again before the time set with
    /// [`Self::set_min_cs_high_ns`] had passed
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
            // a device that has never been selected has been idle for ever
            if let Some(since) = d.deselected_since {
                let idle_for = w.now - since;
                if idle_for < w.min_cs_high {
                    w.cs_high_violations += 1;
                }
                let required = w.min_cs_high.max(w.cs_high);
                if idle_for < required {
                    w.advance(required - idle_for);
                }
            }
            w.devs[self.index].cs_n = false;
        } else if !select && !d.cs_n {
            let now = w.now;
            let d = &mut w.devs[self.index];
            d.cs_n = true;
            d.deselected_since = Some(now);
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
    fn back_to_back_transfers_keep_cs_high_for_one_sck_period() {
        /// Samples CS on a 50 MHz system clock, like a slave with a synchronizer
        struct CsSampler {
            cs_n: bool,
            /// (time of the tick, sampled cs_n) whenever the sampled value changes
            edges: Rc<RefCell<Vec<(u64, bool)>>>,
            sampled: bool,
            now: u64,
        }
        impl SpiPinModel for CsSampler {
            fn set_inputs(&mut self, cs_n: bool, _: bool, _: bool) {
                self.cs_n = cs_n;
            }
            fn period_ps(&self) -> Option<u64> {
                Some(20_000)
            }
            fn tick(&mut self) {
                if self.cs_n != self.sampled {
                    self.sampled = self.cs_n;
                    self.edges.borrow_mut().push((self.now, self.cs_n));
                }
            }
            fn miso(&self) -> Option<bool> {
                None
            }
            fn set_time_ps(&mut self, now_ps: u64) {
                self.now = now_ps;
            }
        }
        let edges = Rc::new(RefCell::new(Vec::new()));
        let bus = SimSpiBus::new();
        let cs = bus.add_device(CsSampler {
            cs_n: true,
            edges: edges.clone(),
            sampled: true,
            now: 0,
        });
        let spi = bus.master(MODE_0, 1_000_000).unwrap();
        let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
        for _ in 0..3 {
            dev.write(&[0x00]).unwrap();
        }
        bus.run_ns(100);

        // three separate frames: low, high, low, high, low, high
        let levels: Vec<bool> = edges.borrow().iter().map(|e| e.1).collect();
        assert_eq!(levels, [false, true, false, true, false, true]);
        // each gap is one SCK period (1 µs), give or take a system clock period
        for w in edges.borrow().windows(2).filter(|w| w[0].1) {
            assert!((w[1].0 - w[0].0).abs_diff(1_000_000) <= 20_000, "{w:?}");
        }
        // the bus's own wait is not a t_CSH violation
        assert_eq!(bus.cs_high_violations(), 0);
    }

    #[test]
    fn a_hand_built_master_gets_the_gap_from_set_cs_high_ns() {
        // CS high time between two back-to-back frames on a master built from the pins
        let gap_ns = |set: Option<u64>| {
            let bus = SimSpiBus::new();
            if let Some(ns) = set {
                bus.set_cs_high_ns(ns);
            }
            let cs = bus.add_device(whoami_pin());
            let spi = BitBangSpi::new(
                bus.sck_pin(),
                bus.mosi_pin(),
                bus.miso_pin(),
                bus.delay(),
                MODE_0,
                1_000_000,
            )
            .unwrap();
            let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
            dev.write(&[0x20, 0x01]).unwrap();
            let deselected = bus.now_ns();
            dev.write(&[0x20, 0x02]).unwrap();
            // the second frame is 16 SCK periods of 1 µs
            bus.now_ns() - deselected - 16_000
        };
        assert_eq!(gap_ns(None), 0);
        assert_eq!(gap_ns(Some(1_000)), 1_000);
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
