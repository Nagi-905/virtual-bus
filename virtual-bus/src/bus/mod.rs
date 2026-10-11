//! Bus infrastructure. Contains no IC models (those are in [`crate::devices`]).
//!
//! - [`i2c`] — I2C transaction-level bus, bit-bang master and signal lines
//! - [`spi`] — the same for SPI
//! - [`pin`] — `InvertedPin` for logical inversion, and `SimPin`, a wire such as IRQ or DRDY
//! - [`SimDelay`] — a `DelayNs` that moves a `SimClock`
//! - [`SimClock`] — simulated time, shared by models and the signal lines that follow it

use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

use embedded_hal::delay::DelayNs;

pub mod i2c;
pub mod pin;
pub mod spi;

/// Shared handle, so that tests can look inside a model after attaching it
pub type Shared<T> = Rc<RefCell<T>>;

/// Creates an `Rc<RefCell<T>>`
pub fn shared<T>(value: T) -> Shared<T> {
    Rc::new(RefCell::new(value))
}

/// A [`DelayNs`] that moves a [`SimClock`]. The same type for every bus
#[derive(Clone)]
pub struct SimDelay {
    pub(crate) clock: SimClock,
}

impl DelayNs for SimDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.clock.advance_ps(u64::from(ns) * 1000);
    }
}

/// Signal lines that follow a clock: they run their models up to the new time when it moves
pub(crate) trait Follower {
    fn run_to(&self, ps: u64);
}

/// Simulated time, one for the whole test.
///
/// - Models read it (`now_ns`) to know if they are still busy
/// - Drivers move it through [`delay`](Self::delay) (blocking and async `DelayNs`)
/// - Signal lines built with `with_clock` share it: moving it runs their models and RTL
/// - Clones share one time. Kept in ps
///
/// ```
/// use embedded_hal::delay::DelayNs;
/// use virtual_bus::SimClock;
///
/// let clock = SimClock::new();
/// let mut delay = clock.delay(); // give this to the driver
/// delay.delay_ms(5);
/// assert_eq!(clock.now_ns(), 5_000_000);
/// ```
#[derive(Clone, Default)]
pub struct SimClock {
    inner: Rc<ClockInner>,
}

#[derive(Default)]
struct ClockInner {
    ps: Cell<u64>,
    followers: RefCell<Vec<Weak<dyn Follower>>>,
}

impl SimClock {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn now_ps(&self) -> u64 {
        self.inner.ps.get()
    }

    pub fn now_ns(&self) -> u64 {
        self.now_ps() / 1000
    }

    /// Moves time forward, e.g. per byte in a model of a chip that is polled without a delay
    pub fn advance_ns(&self, ns: u64) {
        self.advance_ps(ns * 1000);
    }

    /// A `DelayNs` that moves this clock
    pub fn delay(&self) -> SimDelay {
        SimDelay {
            clock: self.clone(),
        }
    }

    /// Moves time and runs every follower up to it. Never call it while holding a follower's lines
    pub(crate) fn advance_ps(&self, dt: u64) {
        let t = self.now_ps() + dt;
        let followers: Vec<_> = {
            let mut f = self.inner.followers.borrow_mut();
            f.retain(|w| w.strong_count() > 0);
            f.iter().filter_map(Weak::upgrade).collect()
        };
        for f in followers {
            f.run_to(t);
        }
        self.inner.ps.set(t);
    }

    pub(crate) fn follow(&self, follower: Weak<dyn Follower>) {
        self.inner.followers.borrow_mut().push(follower);
    }
}
