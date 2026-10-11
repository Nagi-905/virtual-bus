//! A busy model on `VirtualI2cBus`, timed by `SimClock`.
//!
//! - The model NACKs its address for 5 ms after a write, like an EEPROM (t_WR)
//! - The driver code waits with the clock's delay, blocking and async

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorKind, I2c, NoAcknowledgeSource};
use virtual_bus::{Direction, I2cSlave, Nack, SimClock, VirtualI2cBus, block_on};

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
