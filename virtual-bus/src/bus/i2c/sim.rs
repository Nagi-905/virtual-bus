//! Pin-level I2C signal lines (SCL / SDA) and simulated time.
//!
//! The master gets the pins as [`OutputPin`] / [`InputPin`] and the time as
//! [`DelayNs`](embedded_hal::delay::DelayNs). Calling `delay_ns` advances the simulated time by that amount.
//!
//! - Models driven directly by SCL / SDA (such as `i2c_whoami.v` in the examples) are evaluated in
//!   [`I2cPinModel::set_inputs`] whenever a line changes
//! - Models running on a system clock (such as ICs that oversample SCL / SDA) return
//!   [`I2cPinModel::period_ps`] and get [`I2cPinModel::tick`] as time advances
//!
//! SCL / SDA are open-drain: low if anyone pulls them low (wired-AND). When one model pulls SDA,
//! the other models' inputs change too, so the inputs are redelivered until the line levels settle.

use std::cell::RefCell;
use std::convert::Infallible;
use std::rc::Rc;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin};

use super::bitbang::BitBangI2c;
use super::{Direction, I2cSlave};
use crate::bus::SimDelay;

/// Bit-bang I2C master returned by [`SimI2cBus::master`]
pub type SimI2cMaster = BitBangI2c<SimI2cPin, SimI2cPin, SimDelay>;

/// A model attached to the I2C signal lines
pub trait I2cPinModel {
    /// The SCL / SDA levels changed (may also be called when nothing changed)
    fn set_inputs(&mut self, scl: bool, sda: bool);
    /// The system clock period (ps), if the model runs on a system clock
    fn period_ps(&self) -> Option<u64> {
        None
    }
    /// Advances by one rising edge of the system clock
    fn tick(&mut self) {}
    /// Whether the model is pulling SDA low
    fn sda_low(&self) -> bool;
    /// If true, the `sda` passed to `set_inputs` is the level driven by everyone else.
    ///
    /// For DUTs without an output enable, wrapped in a simulation wrapper that resolves the
    /// line itself. Feeding back a level that includes the model's own output would make it
    /// unable to release the line once it pulled it low
    fn wants_external_sda(&self) -> bool {
        false
    }
    /// The simulated time (ps) moved to `now_ps`. Called before any `set_inputs` / `tick`
    /// at that time, and once when the model is attached. For models that dump waveforms
    fn set_time_ps(&mut self, _now_ps: u64) {}
}

/// Models wrapped with [`crate::shared`] can be attached too (to poke at and observe them afterwards)
impl<T: I2cPinModel + ?Sized> I2cPinModel for Rc<RefCell<T>> {
    fn set_inputs(&mut self, scl: bool, sda: bool) {
        self.borrow_mut().set_inputs(scl, sda)
    }
    fn period_ps(&self) -> Option<u64> {
        self.borrow().period_ps()
    }
    fn tick(&mut self) {
        self.borrow_mut().tick()
    }
    fn sda_low(&self) -> bool {
        self.borrow().sda_low()
    }
    fn wants_external_sda(&self) -> bool {
        self.borrow().wants_external_sda()
    }
    fn set_time_ps(&mut self, now_ps: u64) {
        self.borrow_mut().set_time_ps(now_ps)
    }
}

/// A recorded change of the signal lines
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LineEvent {
    pub time_ps: u64,
    pub scl: bool,
    pub sda: bool,
}

struct Slot {
    model: Box<dyn I2cPinModel>,
    period: Option<u64>,
    next: u64,
}

/// How many times the inputs are redelivered at most before the line levels must settle
const MAX_SETTLE_ROUNDS: usize = 64;

struct World {
    now: u64,
    master_scl_low: bool,
    master_sda_low: bool,
    slots: Vec<Slot>,
    last: (bool, bool),
    trace: Option<Vec<LineEvent>>,
}

impl World {
    fn scl(&self) -> bool {
        !self.master_scl_low
    }

    fn sda(&self) -> bool {
        !(self.master_sda_low || self.slots.iter().any(|s| s.model.sda_low()))
    }

    fn record(&mut self) {
        let lines = (self.scl(), self.sda());
        if lines != self.last {
            self.last = lines;
            let now = self.now;
            if let Some(t) = &mut self.trace {
                t.push(LineEvent {
                    time_ps: now,
                    scl: lines.0,
                    sda: lines.1,
                });
            }
        }
    }

    /// The SDA level driven by everyone except model `i` (the master and the other models)
    fn external_sda(&self, i: usize) -> bool {
        !(self.master_sda_low
            || self
                .slots
                .iter()
                .enumerate()
                .any(|(j, o)| j != i && o.model.sda_low()))
    }

    /// Delivers the current line levels to all models, and repeats until no output changes
    fn settle(&mut self) {
        for _ in 0..MAX_SETTLE_ROUNDS {
            let before: Vec<bool> = self.slots.iter().map(|s| s.model.sda_low()).collect();
            let (scl, sda) = (self.scl(), self.sda());
            let inputs: Vec<bool> = (0..self.slots.len())
                .map(|i| {
                    if self.slots[i].model.wants_external_sda() {
                        self.external_sda(i)
                    } else {
                        sda
                    }
                })
                .collect();
            for (s, sda) in self.slots.iter_mut().zip(inputs) {
                s.model.set_inputs(scl, sda);
            }
            if self.slots.iter().map(|s| s.model.sda_low()).eq(before) {
                self.record();
                return;
            }
        }
        panic!("SDA does not settle (a model's output is oscillating)");
    }

    /// Moves the time to `t` and tells the models if it changed
    fn set_now(&mut self, t: u64) {
        if t != self.now {
            self.now = t;
            for s in &mut self.slots {
                s.model.set_time_ps(t);
            }
        }
    }

    fn advance(&mut self, dt: u64) {
        let end = self.now + dt;
        while let Some(t) = self
            .slots
            .iter()
            .filter(|s| s.period.is_some())
            .map(|s| s.next)
            .min()
            .filter(|&t| t <= end)
        {
            self.set_now(t);
            // every model ticking now sees the line levels from before the tick as its inputs
            for s in &mut self.slots {
                if let Some(p) = s.period.filter(|_| s.next == t) {
                    s.model.tick();
                    s.next += p;
                }
            }
            self.settle();
        }
        self.set_now(end);
    }
}

/// I2C signal lines. Cloning gives another handle to the same lines
#[derive(Clone)]
pub struct SimI2cBus {
    world: Rc<RefCell<World>>,
}

impl Default for SimI2cBus {
    fn default() -> Self {
        Self::new()
    }
}

impl SimI2cBus {
    pub fn new() -> Self {
        Self {
            world: Rc::new(RefCell::new(World {
                now: 0,
                master_scl_low: false,
                master_sda_low: false,
                slots: Vec::new(),
                last: (true, true),
                trace: None,
            })),
        }
    }

    /// Attaches a model to the lines. Any number of models can share them
    pub fn attach(&self, mut model: impl I2cPinModel + 'static) {
        let mut w = self.world.borrow_mut();
        model.set_time_ps(w.now);
        let period = model.period_ps();
        assert!(period != Some(0), "period_ps must be greater than 0");
        let next = w.now + period.unwrap_or(0);
        w.slots.push(Slot {
            model: Box::new(model),
            period,
            next,
        });
        w.settle();
    }

    /// The master's SCL
    pub fn scl_pin(&self) -> SimI2cPin {
        SimI2cPin {
            world: self.world.clone(),
            line: Line::Scl,
        }
    }

    /// The master's SDA
    pub fn sda_pin(&self) -> SimI2cPin {
        SimI2cPin {
            world: self.world.clone(),
            line: Line::Sda,
        }
    }

    /// A [`DelayNs`](embedded_hal::delay::DelayNs) that advances the time of these lines
    pub fn delay(&self) -> SimDelay {
        let w = self.world.clone();
        SimDelay::new(Rc::new(move |ps| w.borrow_mut().advance(ps)))
    }

    /// A bit-bang I2C master on these lines
    pub fn master(&self, freq_hz: u32) -> SimI2cMaster {
        BitBangI2c::new(self.scl_pin(), self.sda_pin(), self.delay(), freq_hz)
    }

    /// Just lets time pass
    pub fn run_ns(&self, ns: u64) {
        self.world.borrow_mut().advance(ns * 1000);
    }

    pub fn now_ps(&self) -> u64 {
        self.world.borrow().now
    }

    pub fn now_ns(&self) -> u64 {
        self.now_ps() / 1000
    }

    pub fn scl(&self) -> bool {
        self.world.borrow().scl()
    }

    pub fn sda(&self) -> bool {
        self.world.borrow().sda()
    }

    /// Starts recording line changes (discarding anything recorded so far)
    pub fn enable_trace(&self) {
        self.world.borrow_mut().trace = Some(Vec::new());
    }

    /// The recorded line changes
    pub fn trace(&self) -> Vec<LineEvent> {
        self.world.borrow().trace.clone().unwrap_or_default()
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Line {
    Scl,
    Sda,
}

/// The master's open-drain pin. `set_low` pulls the line low, `set_high` releases it.
/// `is_high` returns the actual level of the line
pub struct SimI2cPin {
    world: Rc<RefCell<World>>,
    line: Line,
}

impl ErrorType for SimI2cPin {
    type Error = Infallible;
}

impl SimI2cPin {
    fn drive(&mut self, low: bool) {
        let mut w = self.world.borrow_mut();
        match self.line {
            Line::Scl => w.master_scl_low = low,
            Line::Sda => w.master_sda_low = low,
        }
        w.settle();
    }
}

impl OutputPin for SimI2cPin {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.drive(true);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Infallible> {
        self.drive(false);
        Ok(())
    }
}

impl InputPin for SimI2cPin {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        let w = self.world.borrow();
        Ok(match self.line {
            Line::Scl => w.scl(),
            Line::Sda => w.sda(),
        })
    }

    fn is_low(&mut self) -> Result<bool, Infallible> {
        self.is_high().map(|h| !h)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PinState {
    Idle,
    Addr,
    AddrAck,
    WriteData,
    WriteAck,
    ReadData,
    ReadAck,
    /// Not addressed to us, or we NACKed. Wait for the next START / STOP
    Ignore,
}

/// Adapter that runs a transaction-level [`I2cSlave`] at pin level.
///
/// Like typical RTL, it samples SCL / SDA with a system clock and detects edges.
/// Use it to test an IC without RTL through the bit-bang master path,
/// or to put a Rust model on the same lines as RTL.
pub struct PinLevelI2cSlave<S> {
    slave: S,
    address: u8,
    period: u64,
    inputs: (bool, bool),
    prev: (bool, bool),
    state: PinState,
    bits: u8,
    shift: u8,
    tx: u8,
    read: bool,
    master_ack: bool,
    active: bool,
    sda_low: bool,
}

impl<S: I2cSlave> PinLevelI2cSlave<S> {
    /// `address` is the 7-bit address. The system clock is 50 MHz
    pub fn new(address: u8, slave: S) -> Self {
        Self {
            slave,
            address,
            period: 20_000,
            inputs: (true, true),
            prev: (true, true),
            state: PinState::Idle,
            bits: 0,
            shift: 0,
            tx: 0,
            read: false,
            master_ack: false,
            active: false,
            sda_low: false,
        }
    }

    /// Changes the system clock period
    pub fn with_period_ps(mut self, period_ps: u64) -> Self {
        self.period = period_ps;
        self
    }

    pub fn slave(&self) -> &S {
        &self.slave
    }

    fn load_tx(&mut self) {
        let mut b = [0u8];
        self.slave.read(&mut b);
        self.tx = b[0];
        self.sda_low = self.tx & 0x80 == 0;
        self.bits = 1;
        self.state = PinState::ReadData;
    }
}

impl<S: I2cSlave> I2cPinModel for PinLevelI2cSlave<S> {
    fn set_inputs(&mut self, scl: bool, sda: bool) {
        // a synchronous circuit: remember the inputs and look at them on the system clock
        self.inputs = (scl, sda);
    }

    fn period_ps(&self) -> Option<u64> {
        Some(self.period)
    }

    fn tick(&mut self) {
        let (scl, sda) = self.inputs;
        let (pscl, psda) = self.prev;
        self.prev = (scl, sda);
        let rise = scl && !pscl;
        let fall = !scl && pscl;

        if scl && pscl && psda && !sda {
            // START / repeated START
            self.state = PinState::Addr;
            self.bits = 0;
            self.sda_low = false;
            return;
        }
        if scl && pscl && !psda && sda {
            // STOP
            if self.active {
                self.slave.stop();
                self.active = false;
            }
            self.state = PinState::Idle;
            self.sda_low = false;
            return;
        }

        match self.state {
            PinState::Idle | PinState::Ignore => {}
            PinState::Addr => {
                if rise {
                    self.shift = (self.shift << 1) | u8::from(sda);
                    self.bits += 1;
                } else if fall && self.bits == 8 {
                    if self.shift >> 1 == self.address {
                        self.read = self.shift & 1 != 0;
                        let dir = if self.read {
                            Direction::Read
                        } else {
                            Direction::Write
                        };
                        self.active = true;
                        self.slave.start(dir);
                        self.sda_low = true;
                        self.state = PinState::AddrAck;
                    } else {
                        self.state = PinState::Ignore;
                    }
                }
            }
            PinState::AddrAck => {
                if fall {
                    if self.read {
                        self.load_tx();
                    } else {
                        self.sda_low = false;
                        self.bits = 0;
                        self.state = PinState::WriteData;
                    }
                }
            }
            PinState::WriteData => {
                if rise {
                    self.shift = (self.shift << 1) | u8::from(sda);
                    self.bits += 1;
                } else if fall && self.bits == 8 {
                    if self.slave.write(&[self.shift]).is_ok() {
                        self.sda_low = true;
                        self.state = PinState::WriteAck;
                    } else {
                        self.state = PinState::Ignore;
                    }
                }
            }
            PinState::WriteAck => {
                if fall {
                    self.sda_low = false;
                    self.bits = 0;
                    self.state = PinState::WriteData;
                }
            }
            PinState::ReadData => {
                if fall {
                    if self.bits == 8 {
                        self.sda_low = false;
                        self.state = PinState::ReadAck;
                    } else {
                        self.sda_low = self.tx & (0x80 >> self.bits) == 0;
                        self.bits += 1;
                    }
                }
            }
            PinState::ReadAck => {
                if rise {
                    self.master_ack = !sda;
                } else if fall {
                    if self.master_ack {
                        self.load_tx();
                    } else {
                        self.state = PinState::Ignore;
                    }
                }
            }
        }
    }

    fn sda_low(&self) -> bool {
        self.sda_low
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::VirtualI2cBus;
    use crate::devices::register::{I2cFormat, I2cRegisterDevice, RegisterMap};
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::{ErrorKind, I2c, NoAcknowledgeSource};

    /// 0x0F = 0xA5 (read-only), 0x10..=0x11 are read-write registers
    fn regs() -> RegisterMap {
        RegisterMap::new().ro(0x0F, 0xA5).rw(0x10, 0).rw(0x11, 0)
    }

    fn device(map: &RegisterMap) -> I2cRegisterDevice {
        map.i2c(I2cFormat::new())
    }

    #[test]
    fn delay_advances_time() {
        let bus = SimI2cBus::new();
        let mut d = bus.delay();
        d.delay_us(3);
        d.delay_ns(250);
        assert_eq!(bus.now_ns(), 3250);
    }

    #[test]
    fn models_are_told_when_time_moves() {
        use crate::shared;
        #[derive(Default)]
        struct Clock {
            times: Vec<u64>,
            ticks_at: Vec<u64>,
        }
        impl I2cPinModel for Clock {
            fn set_inputs(&mut self, _: bool, _: bool) {}
            fn period_ps(&self) -> Option<u64> {
                Some(400)
            }
            fn tick(&mut self) {
                let now = *self.times.last().unwrap();
                self.ticks_at.push(now);
            }
            fn sda_low(&self) -> bool {
                false
            }
            fn set_time_ps(&mut self, now_ps: u64) {
                self.times.push(now_ps);
            }
        }
        let m = shared(Clock::default());
        let bus = SimI2cBus::new();
        bus.run_ns(1);
        bus.attach(m.clone());
        bus.delay().delay_ns(1);
        let m = m.borrow();
        // attach, the two ticks, and the end of the delay
        assert_eq!(m.times, [1000, 1400, 1800, 2000]);
        // a tick sees the time it happens at
        assert_eq!(m.ticks_at, [1400, 1800]);
    }

    #[test]
    fn open_drain_lines_are_wired_and() {
        struct Holder;
        impl I2cPinModel for Holder {
            fn set_inputs(&mut self, _: bool, _: bool) {}
            fn sda_low(&self) -> bool {
                true
            }
        }
        let bus = SimI2cBus::new();
        let mut sda = bus.sda_pin();
        assert!(sda.is_high().unwrap());
        bus.attach(Holder);
        sda.set_high().unwrap();
        assert!(sda.is_low().unwrap());
        assert!(bus.scl());
    }

    #[test]
    fn models_wanting_external_sda_do_not_see_their_own_output() {
        /// Pulls SDA low while `pull` is set, and records the SDA level it is given
        struct Probe {
            external: bool,
            pull: Rc<RefCell<bool>>,
            seen: Rc<RefCell<bool>>,
        }
        impl I2cPinModel for Probe {
            fn set_inputs(&mut self, _: bool, sda: bool) {
                *self.seen.borrow_mut() = sda;
            }
            fn sda_low(&self) -> bool {
                *self.pull.borrow()
            }
            fn wants_external_sda(&self) -> bool {
                self.external
            }
        }
        let probe = |external| {
            let (pull, seen) = (Rc::new(RefCell::new(false)), Rc::new(RefCell::new(true)));
            let p = Probe {
                external,
                pull: pull.clone(),
                seen: seen.clone(),
            };
            (p, pull, seen)
        };
        let bus = SimI2cBus::new();
        let (ext, ext_pull, ext_seen) = probe(true);
        let (plain, _, plain_seen) = probe(false);
        bus.attach(ext);
        bus.attach(plain);
        let mut sda = bus.sda_pin();

        // the external model pulls SDA: the line is low, but it is given the level without itself
        *ext_pull.borrow_mut() = true;
        sda.set_high().unwrap(); // the master releases SDA; this delivers the inputs again
        assert!(!bus.sda());
        assert!(*ext_seen.borrow());
        assert!(!*plain_seen.borrow());

        // the master pulls SDA too: now someone else drives it low
        sda.set_low().unwrap();
        assert!(!*ext_seen.borrow());

        // everybody releases: high for both
        *ext_pull.borrow_mut() = false;
        sda.set_high().unwrap();
        assert!(bus.sda());
        assert!(*ext_seen.borrow() && *plain_seen.borrow());
    }

    #[test]
    fn bitbang_master_talks_to_pin_level_model() {
        let map = regs();
        let bus = SimI2cBus::new();
        bus.attach(PinLevelI2cSlave::new(0x20, device(&map)));
        let mut i2c = bus.master(400_000);

        i2c.write(0x20, &[0x10, 0x5A, 0xC3]).unwrap();
        assert_eq!((map.get(0x10), map.get(0x11)), (0x5A, 0xC3));

        map.set(0x11, 0x02); // change the value on the model side
        let mut b = [0u8; 3];
        i2c.write_read(0x20, &[0x0F], &mut b).unwrap();
        assert_eq!(b, [0xA5, 0x5A, 0x02]);
    }

    #[test]
    fn wrong_address_is_nacked_at_pin_level() {
        let bus = SimI2cBus::new();
        bus.attach(PinLevelI2cSlave::new(0x20, device(&regs())));
        let mut i2c = bus.master(400_000);
        assert_eq!(
            i2c.write(0x21, &[0]),
            Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
        );
        // the bus is released after the failure
        assert!(bus.scl() && bus.sda());
        i2c.write(0x20, &[0]).unwrap();
    }

    #[test]
    fn bus_speed_follows_master_delays() {
        let bus = SimI2cBus::new();
        bus.attach(PinLevelI2cSlave::new(0x20, device(&regs())));
        let mut i2c = bus.master(100_000);
        bus.enable_trace();
        i2c.write(0x20, &[0x00]).unwrap();
        let rises: Vec<u64> = bus
            .trace()
            .windows(2)
            .filter(|w| !w[0].scl && w[1].scl)
            .map(|w| w[1].time_ps)
            .collect();
        // 9 clocks x 2 bytes + STOP
        assert!(rises.len() >= 18);
        let period_ns = (rises[2] - rises[1]) / 1000;
        assert_eq!(period_ns, 10_000);
    }

    #[test]
    fn pin_level_and_transaction_level_mix_on_one_virtual_bus() {
        // 0x20 is transaction level, 0x21 goes through the pin-level path
        let fast = regs();
        let slow = regs();
        let lines = SimI2cBus::new();
        lines.attach(PinLevelI2cSlave::new(0x21, device(&slow)));

        let mut bus = VirtualI2cBus::new();
        bus.attach(0x20, device(&fast)).unwrap();
        bus.attach_i2c(0x21, lines.master(400_000)).unwrap();

        for addr in [0x20, 0x21] {
            bus.write(addr, &[0x10, addr]).unwrap();
        }
        assert_eq!(fast.get(0x10), 0x20);
        assert_eq!(slow.get(0x10), 0x21);
        assert!(lines.now_ns() > 0);
    }
}
