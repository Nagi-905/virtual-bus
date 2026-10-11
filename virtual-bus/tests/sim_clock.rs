//! `SimClock`: time for models, delays and signal lines.
//!
//! - A busy model on `VirtualI2cBus` (an EEPROM that NACKs for t_WR), waited for by blocking and async code
//! - Clones share one time
//! - Moving the clock from anywhere runs the models on every signal line that follows it

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorKind, I2c, NoAcknowledgeSource};
use virtual_bus::bus::i2c::sim::{I2cPinModel, SimI2cBus};
use virtual_bus::bus::spi::sim::{SimSpiBus, SpiPinModel};
use virtual_bus::{Direction, I2cSlave, Nack, SimClock, VirtualI2cBus, block_on, shared};

const ADDR: u8 = 0x50;
const T_WR_NS: u64 = 5_000_000;

struct Eeprom {
    clock: SimClock,
    busy_until_ns: u64,
    wrote: bool,
}

impl I2cSlave for Eeprom {
    fn start(&mut self, _: Direction) -> Result<(), Nack> {
        if self.clock.now_ns() < self.busy_until_ns {
            return Err(Nack);
        }
        Ok(())
    }

    fn write(&mut self, _: &[u8]) -> Result<(), Nack> {
        self.wrote = true;
        Ok(())
    }

    fn read(&mut self, _: &mut [u8]) {}

    fn stop(&mut self) {
        if std::mem::take(&mut self.wrote) {
            self.busy_until_ns = self.clock.now_ns() + T_WR_NS;
        }
    }
}

fn setup() -> (VirtualI2cBus, SimClock) {
    let clock = SimClock::new();
    let mut bus = VirtualI2cBus::new();
    let eeprom = Eeprom {
        clock: clock.clone(),
        busy_until_ns: 0,
        wrote: false,
    };
    bus.attach(ADDR, eeprom).unwrap();
    (bus, clock)
}

const BUSY: Result<(), ErrorKind> = Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));

#[test]
fn busy_until_the_driver_waits() {
    let (mut bus, clock) = setup();
    let mut delay = clock.delay();

    bus.write(ADDR, &[0x00, 0x42]).unwrap();
    assert_eq!(bus.write(ADDR, &[0x00]), BUSY);

    delay.delay_ms(4);
    assert_eq!(bus.write(ADDR, &[0x00]), BUSY);
    delay.delay_ms(1);
    assert_eq!(bus.write(ADDR, &[0x00]), Ok(()));
    assert_eq!(clock.now_ns(), T_WR_NS);
}

/// Ack polling, as async drivers do it: retry every 1 ms until the chip answers
async fn write_and_wait<I, D>(i2c: &mut I, delay: &mut D, data: &[u8]) -> u32
where
    I: embedded_hal_async::i2c::I2c,
    D: embedded_hal_async::delay::DelayNs,
{
    i2c.write(ADDR, data).await.ok();
    let mut tries = 0;
    while i2c.write(ADDR, &[]).await.is_err() {
        tries += 1;
        delay.delay_ms(1).await;
    }
    tries
}

#[test]
fn async_driver_waits_on_the_same_clock() {
    let (mut bus, clock) = setup();
    let mut delay = clock.delay();

    let tries = block_on(write_and_wait(&mut bus, &mut delay, &[0x00, 0x42]));
    assert_eq!(tries, 5);
    assert_eq!(clock.now_ns(), T_WR_NS);
}

#[test]
fn clones_share_one_time() {
    let clock = SimClock::new();
    let other = clock.clone();
    clock.advance_ns(250);
    other.delay().delay_us(1);
    assert_eq!((clock.now_ns(), other.now_ns()), (1_250, 1_250));
    assert_eq!(clock.now_ps(), 1_250_000);
}

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
