//! Bus infrastructure. Contains no IC models (those are in [`crate::devices`]).
//!
//! - [`i2c`] — I2C transaction-level bus, bit-bang master and signal lines
//! - [`spi`] — the same for SPI
//! - [`pin`] — pin adapters (`InvertedPin` for logical inversion)
//! - [`SimDelay`] — a `DelayNs` that advances the simulated time of the signal lines

use std::cell::RefCell;
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
