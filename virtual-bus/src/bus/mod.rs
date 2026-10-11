//! Bus infrastructure. Contains no IC models (those are in [`crate::devices`]).
//!
//! - [`i2c`] — I2C transaction-level bus, bit-bang master and signal lines
//! - [`spi`] — the same for SPI
//! - [`pin`] — pin adapters (`InvertedPin` for logical inversion)
//! - [`SimDelay`] — a `DelayNs` that advances the simulated time of the signal lines
//! - [`SimClock`] — simulated time for models on the transaction-level buses

use std::cell::{Cell, RefCell};
use std::rc::Rc;

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

/// A [`DelayNs`] that advances simulated time. The same type for both I2C and SPI signal lines
#[derive(Clone)]
pub struct SimDelay {
    advance_ps: Rc<dyn Fn(u64)>,
}

impl SimDelay {
    pub(crate) fn new(advance_ps: Rc<dyn Fn(u64)>) -> Self {
        Self { advance_ps }
    }
}

impl DelayNs for SimDelay {
    fn delay_ns(&mut self, ns: u32) {
        (self.advance_ps)(u64::from(ns) * 1000);
    }
}

/// Simulated time for models on [`VirtualI2cBus`](crate::VirtualI2cBus) and the other
/// transaction-level buses.
///
/// - Models read it (`now_ns`) to know if they are still busy
/// - Drivers move it through [`delay`](Self::delay) (blocking and async `DelayNs`)
/// - Clones share one time. The bus itself takes no time
/// - Kept in ps, like the signal lines
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
    ps: Rc<Cell<u64>>,
}

impl SimClock {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn now_ps(&self) -> u64 {
        self.ps.get()
    }

    pub fn now_ns(&self) -> u64 {
        self.now_ps() / 1000
    }

    /// Moves time forward, e.g. per byte in a model of a chip that is polled without a delay
    pub fn advance_ns(&self, ns: u64) {
        self.ps.set(self.ps.get() + ns * 1000);
    }

    /// A `DelayNs` that moves this clock
    pub fn delay(&self) -> SimDelay {
        let ps = self.ps.clone();
        SimDelay::new(Rc::new(move |dt| ps.set(ps.get() + dt)))
    }
}
