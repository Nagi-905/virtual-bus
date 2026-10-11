//! One `SimClock` for a Rust model on `VirtualI2cBus` and RTL on the SPI signal lines.
//!
//! - The RTL is `spi_counter.v`, a timer on a 50 MHz system clock
//! - The Rust model is an EEPROM that NACKs its address for 1 ms after a write
//! - One delay moves both; the async test handles the timer's `irq` through a `SimPin`

use std::cell::Cell;

use embassy_futures::join::join;
use embedded_hal::i2c::{ErrorKind, I2c, NoAcknowledgeSource};
use embedded_hal::spi::{MODE_0, SpiDevice};
use embedded_hal_async::digital::Wait;
use embedded_hal_bus::spi::ExclusiveDevice;
use rtl_demo::spi_counter::SpiCounter;
use virtual_bus::bus::SimDelay;
use virtual_bus::bus::spi::sim::{SimCsPin, SimSpiBus, SimSpiMaster};
use virtual_bus::{
    Direction, I2cSlave, Nack, Shared, SimClock, SimPin, VirtualI2cBus, block_on, shared,
};

const EEPROM: u8 = 0x50;
const T_WR_NS: u64 = 1_000_000;
const BUSY: Result<(), ErrorKind> = Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));

/// NACKs its address for 1 ms after a write
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

fn eeprom_on(clock: &SimClock) -> VirtualI2cBus {
    let mut i2c = VirtualI2cBus::new();
    let eeprom = Eeprom {
        clock: clock.clone(),
        busy_until_ns: 0,
        wrote: false,
    };
    i2c.attach(EEPROM, eeprom).unwrap();
    i2c
}

type Counter = ExclusiveDevice<SimSpiMaster, SimCsPin, SimDelay>;

/// The timer RTL on SPI lines that follow `clock`, through a 1 MHz mode 0 master
fn counter_on(clock: &SimClock) -> (Counter, Shared<SpiCounter>) {
    let lines = SimSpiBus::with_clock(clock);
    let rtl = shared(SpiCounter::new());
    let cs = lines.add_device(rtl.clone());
    let spi = lines.master(MODE_0, 1_000_000).unwrap();
    (ExclusiveDevice::new(spi, cs, lines.delay()).unwrap(), rtl)
}

fn read(spi: &mut Counter, reg: u8) -> u8 {
    let mut b = [0x80 | reg, 0];
    spi.transfer_in_place(&mut b).unwrap();
    b[1]
}

/// Starts the timer: COUNT goes up every 256 system clocks (5.12 µs)
fn start(spi: &mut Counter, compare: u8, irq: bool) {
    spi.write(&[SpiCounter::REG_PRESCALE, 255]).unwrap();
    spi.write(&[SpiCounter::REG_COMPARE, compare]).unwrap();
    let irq_en = if irq { SpiCounter::CTRL_IRQ_EN } else { 0 };
    let ctrl = SpiCounter::CTRL_EN | SpiCounter::CTRL_CLR | irq_en;
    spi.write(&[SpiCounter::REG_CTRL, ctrl]).unwrap();
}

#[test]
fn one_delay_times_the_eeprom_and_the_rtl_timer() {
    let clock = SimClock::new();
    let (mut spi, _rtl) = counter_on(&clock);
    let mut i2c = eeprom_on(&clock);

    start(&mut spi, 0xFF, false);
    i2c.write(EEPROM, &[0x00, 0x42]).unwrap();
    assert_eq!(i2c.write(EEPROM, &[0x00]), BUSY);

    embedded_hal::delay::DelayNs::delay_ms(&mut clock.delay(), 1);

    // the EEPROM finished its write cycle, and the RTL counted the same 1 ms
    assert_eq!(i2c.write(EEPROM, &[0x00]), Ok(()));
    let count = read(&mut spi, SpiCounter::REG_COUNT);
    // 1 ms / 5.12 µs = 195, plus the read frame
    assert!((195..=197).contains(&count), "COUNT {count}");
}

/// The whole interrupt flow: wait, read STATUS, clear MATCH, and irq goes low
#[test]
fn firmware_handles_the_rtl_interrupt_to_the_end() {
    let clock = SimClock::new();
    let (mut spi, rtl) = counter_on(&clock);
    let irq = SimPin::new(false);
    let mut irq_pin = irq.clone();
    let mut sample = clock.delay();
    let (started, woke) = (Cell::new(0), Cell::new(0));

    let ((status, cleared), ()) = block_on(join(
        // the firmware's interrupt handling
        async {
            start(&mut spi, 100, true);
            started.set(clock.now_ns());
            irq_pin.wait_for_high().await.unwrap();
            woke.set(clock.now_ns());
            let status = read(&mut spi, SpiCounter::REG_STATUS);
            spi.write(&[SpiCounter::REG_STATUS, 0x01]).unwrap(); // write 1 to clear MATCH
            let cleared = read(&mut spi, SpiCounter::REG_STATUS);
            (status, cleared)
        },
        // the board: irq wired to the MCU pin, sampled every 10 µs
        async {
            while !rtl.borrow().irq() {
                embedded_hal_async::delay::DelayNs::delay_us(&mut sample, 10).await;
            }
            irq.set(true);
        },
    ));

    // MATCH at COUNT 100: 100 × 5.12 µs = 512 µs, seen at the next 10 µs sample
    let waited = woke.get() - started.get();
    assert!((512_000..=522_000).contains(&waited), "waited {waited} ns");
    assert_eq!(status & 0x01, 0x01, "MATCH was set");
    assert_eq!(cleared & 0x01, 0x00, "MATCH is cleared");
    assert!(!rtl.borrow().irq(), "irq follows MATCH");
    // and stays low: COUNT is past COMPARE until it wraps (256 × 5.12 µs)
    embedded_hal::delay::DelayNs::delay_us(&mut clock.delay(), 100);
    assert!(!rtl.borrow().irq());
}
