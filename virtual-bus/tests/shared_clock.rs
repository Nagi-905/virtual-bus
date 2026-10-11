//! One `SimClock` for transaction-level models and signal lines.
//!
//! - Moving the clock from anywhere runs the system clocks of every model on the lines
//! - A Rust model on `VirtualI2cBus` and a model on the signal lines see the same time

use embedded_hal::delay::DelayNs;
use virtual_bus::bus::i2c::sim::{I2cPinModel, SimI2cBus};
use virtual_bus::bus::spi::sim::{SimSpiBus, SpiPinModel};
use virtual_bus::{SimClock, shared};

/// Counts system clock ticks (100 MHz)
#[derive(Default)]
struct Ticks {
    n: u64,
    last_ps: u64,
}

impl I2cPinModel for Ticks {
    fn set_inputs(&mut self, _: bool, _: bool) {}
    fn period_ps(&self) -> Option<u64> {
        Some(10_000)
    }
    fn tick(&mut self) {
        self.n += 1;
    }
    fn sda_low(&self) -> bool {
        false
    }
    fn set_time_ps(&mut self, now_ps: u64) {
        self.last_ps = now_ps;
    }
}

impl SpiPinModel for Ticks {
    fn set_inputs(&mut self, _: bool, _: bool, _: bool) {}
    fn period_ps(&self) -> Option<u64> {
        Some(10_000)
    }
    fn tick(&mut self) {
        self.n += 1;
    }
    fn miso(&self) -> Option<bool> {
        None
    }
    fn set_time_ps(&mut self, now_ps: u64) {
        self.last_ps = now_ps;
    }
}

#[test]
fn any_delay_runs_every_line_on_the_clock() {
    let clock = SimClock::new();
    let i2c = SimI2cBus::with_clock(&clock);
    let spi = SimSpiBus::with_clock(&clock);
    let (a, b) = (shared(Ticks::default()), shared(Ticks::default()));
    i2c.attach(a.clone());
    let _cs = spi.add_device(b.clone());

    // a delay that knows nothing about the lines
    clock.delay().delay_us(1);
    assert_eq!((a.borrow().n, b.borrow().n), (100, 100));

    // the I2C lines' own delay moves the SPI lines too
    i2c.delay().delay_us(1);
    assert_eq!((a.borrow().n, b.borrow().n), (200, 200));

    // a model's advance_ns as well
    clock.advance_ns(500);
    assert_eq!((a.borrow().n, b.borrow().n), (250, 250));

    assert_eq!(clock.now_ns(), 2_500);
    assert_eq!((i2c.now_ns(), spi.now_ns()), (2_500, 2_500));
    assert_eq!(
        (a.borrow().last_ps, b.borrow().last_ps),
        (2_500_000, 2_500_000)
    );
}

#[test]
fn lines_built_later_start_at_the_current_time() {
    let clock = SimClock::new();
    clock.advance_ns(1_000);
    let i2c = SimI2cBus::with_clock(&clock);
    let a = shared(Ticks::default());
    i2c.attach(a.clone());
    assert_eq!(a.borrow().last_ps, 1_000_000);
    clock.advance_ns(100);
    assert_eq!(a.borrow().n, 10);
}

#[test]
fn separate_clocks_stay_separate() {
    let i2c = SimI2cBus::new();
    let other = SimClock::new();
    let a = shared(Ticks::default());
    i2c.attach(a.clone());
    other.advance_ns(1_000);
    assert_eq!(a.borrow().n, 0);
    assert_eq!(i2c.now_ns(), 0);
}
