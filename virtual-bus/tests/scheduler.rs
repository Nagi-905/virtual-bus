//! `block_on` as a SystemVerilog-style scheduler.
//!
//! - Async delays stop the task (`#delay`); tasks resume in time order
//! - `SimPin` waits (`wait`, `@(posedge)`), missing edges from before the wait
//! - A dropped delay leaves no timer behind
//! - Moving the clock runs the signal lines on it

use std::cell::RefCell;
use std::panic::{AssertUnwindSafe, catch_unwind};

use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use virtual_bus::bus::i2c::sim::{I2cPinModel, SimI2cBus};
use virtual_bus::devices::register::{I2cFormat, RegisterMap};
use virtual_bus::{SimClock, SimPin, VirtualI2cBus, block_on, shared};

// --- code shared with the firmware ------------------------------------------

const ADDR: u8 = 0x48;
const CTRL: u8 = 0x20;
const DATA: u8 = 0x28;

async fn start<I: I2c>(i2c: &mut I) -> Result<(), I::Error> {
    i2c.write(ADDR, &[CTRL, 0x01]).await
}

async fn read_when_ready<I: I2c, P: Wait>(i2c: &mut I, drdy: &mut P) -> Result<u16, I::Error> {
    drdy.wait_for_high().await.ok();
    let mut b = [0u8; 2];
    i2c.write_read(ADDR, &[DATA], &mut b).await?;
    Ok(u16::from_be_bytes(b))
}

// --- time order ---------------------------------------------------------------

#[test]
fn tasks_resume_in_time_order() {
    let clock = SimClock::new();
    let log = RefCell::new(Vec::new());
    let (mut da, mut db) = (clock.delay(), clock.delay());
    block_on(join(
        async {
            da.delay_ms(2).await;
            log.borrow_mut().push(("A", clock.now_ns()));
        },
        async {
            db.delay_ms(1).await;
            log.borrow_mut().push(("B", clock.now_ns()));
        },
    ));
    assert_eq!(*log.borrow(), [("B", 1_000_000), ("A", 2_000_000)]);
}

/// Two timers due at the same time: the clock moves once, and the tasks run in the order they waited
#[test]
fn timers_due_at_the_same_time() {
    let clock = SimClock::new();
    let log = RefCell::new(Vec::new());
    let (mut da, mut db) = (clock.delay(), clock.delay());
    block_on(join(
        async {
            da.delay_ms(1).await;
            log.borrow_mut().push(("A", clock.now_ns()));
        },
        async {
            db.delay_ms(1).await;
            log.borrow_mut().push(("B", clock.now_ns()));
        },
    ));
    assert_eq!(*log.borrow(), [("A", 1_000_000), ("B", 1_000_000)]);
    assert_eq!(clock.now_ns(), 1_000_000);
}

/// A blocking delay inside a task holds up the others, as on a real executor
#[test]
fn a_blocking_delay_holds_up_the_other_tasks() {
    let clock = SimClock::new();
    let log = RefCell::new(Vec::new());
    let (mut blocking, mut da) = (clock.delay(), clock.delay());
    block_on(join(
        async {
            DelayNs::delay_ms(&mut da, 2).await; // starts waiting at 0, due at 2 ms
            log.borrow_mut().push(("A", clock.now_ns()));
        },
        async {
            // a blocking driver inside an async task
            embedded_hal::delay::DelayNs::delay_ms(&mut blocking, 5);
            log.borrow_mut().push(("blocking", clock.now_ns()));
        },
    ));
    // A was due at 2 ms but could only run once the blocking delay gave the executor back
    assert_eq!(*log.borrow(), [("blocking", 5_000_000), ("A", 5_000_000)]);
}

// --- fork ... join: firmware and a sensor -------------------------------------

#[test]
fn firmware_waits_for_drdy_from_the_sensor() {
    let regs = RegisterMap::new()
        .rw(CTRL, 0)
        .ro(DATA, 0x12)
        .ro(DATA + 1, 0x34);
    let mut bus = VirtualI2cBus::new();
    bus.attach(ADDR, regs.i2c(I2cFormat::new())).unwrap();
    let clock = SimClock::new();
    let drdy = SimPin::new(false);
    let mut pin = drdy.clone();
    let mut sensor_delay = clock.delay();

    let (value, ()) = block_on(join(
        // the firmware: the same code as on the board
        async {
            start(&mut bus).await.unwrap();
            read_when_ready(&mut bus, &mut pin).await
        },
        // the sensor: converts for 1 ms after start, then raises DRDY
        async {
            while regs.get(CTRL) == 0 {
                sensor_delay.delay_us(10).await;
            }
            sensor_delay.delay_ms(1).await;
            drdy.set(true);
        },
    ));
    assert_eq!(value, Ok(0x1234));
    // join runs the firmware first: start at 0, data ready 1 ms later
    assert_eq!(clock.now_ns(), 1_000_000);
}

// --- SimPin --------------------------------------------------------------------

#[test]
fn a_level_that_is_already_there_is_seen() {
    let mut pin = SimPin::new(false);
    pin.set(true);
    block_on(pin.wait_for_high()).unwrap();
}

#[test]
#[should_panic(expected = "nothing here will wake it")]
fn an_edge_before_the_wait_is_missed() {
    let mut pin = SimPin::new(false);
    pin.set(true); // the edge comes before the driver starts waiting
    let _ = block_on(pin.wait_for_rising_edge());
}

#[test]
fn edges_after_the_wait_are_seen() {
    let clock = SimClock::new();
    let irq = SimPin::new(true);
    let mut pin = irq.clone();
    let mut d = clock.delay();
    block_on(join(pin.wait_for_falling_edge(), async {
        d.delay_us(3).await;
        irq.set(false);
    }))
    .0
    .unwrap();
    assert_eq!(clock.now_ns(), 3_000);
}

#[test]
fn a_rising_edge_after_the_wait_is_seen() {
    let clock = SimClock::new();
    let irq = SimPin::new(false);
    let mut pin = irq.clone();
    let mut d = clock.delay();
    block_on(join(pin.wait_for_rising_edge(), async {
        d.delay_us(2).await;
        irq.set(true);
    }))
    .0
    .unwrap();
    assert_eq!(clock.now_ns(), 2_000);
}

#[test]
fn any_edge_wakes_on_either_direction() {
    let clock = SimClock::new();
    let line = SimPin::new(true);
    let mut pin = line.clone();
    let mut d = clock.delay();
    let woke = RefCell::new(Vec::new());
    block_on(join(
        async {
            pin.wait_for_any_edge().await.unwrap();
            woke.borrow_mut().push(clock.now_ns());
            pin.wait_for_any_edge().await.unwrap();
            woke.borrow_mut().push(clock.now_ns());
        },
        async {
            d.delay_us(1).await;
            line.set(false);
            d.delay_us(1).await;
            line.set(true);
        },
    ));
    assert_eq!(*woke.borrow(), [1_000, 2_000]);
}

/// Several tasks waiting on one wire: one change wakes them all
#[test]
fn one_change_wakes_every_waiter() {
    let clock = SimClock::new();
    let irq = SimPin::new(false);
    let (mut level, mut edge) = (irq.clone(), irq.clone());
    let mut d = clock.delay();
    let woke = RefCell::new(Vec::new());
    block_on(join(
        join(
            async {
                level.wait_for_high().await.unwrap();
                woke.borrow_mut().push(("level", clock.now_ns()));
            },
            async {
                edge.wait_for_rising_edge().await.unwrap();
                woke.borrow_mut().push(("edge", clock.now_ns()));
            },
        ),
        async {
            d.delay_us(3).await;
            irq.set(true);
        },
    ));
    assert_eq!(*woke.borrow(), [("level", 3_000), ("edge", 3_000)]);
}

/// The wrong direction, or driving the level it already has, is not an edge
#[test]
#[should_panic(expected = "nothing here will wake it")]
fn a_falling_edge_does_not_end_a_rising_wait() {
    let clock = SimClock::new();
    let line = SimPin::new(true);
    let mut pin = line.clone();
    let mut d = clock.delay();
    let _ = block_on(join(pin.wait_for_rising_edge(), async {
        d.delay_us(1).await;
        line.set(true); // no change
        line.set(false); // falling
    }));
}

// --- a timeout that loses leaves nothing behind ---------------------------------

#[test]
fn a_dropped_delay_leaves_no_timer() {
    let clock = SimClock::new();
    let drdy = SimPin::new(false);
    let mut pin = drdy.clone();
    let (mut timeout, mut sensor) = (clock.delay(), clock.delay());

    // wait for DRDY with a 100 ms timeout; DRDY comes at 1 ms
    let (got, ()) = block_on(join(
        select(pin.wait_for_high(), timeout.delay_ms(100)),
        async {
            sensor.delay_ms(1).await;
            drdy.set(true);
        },
    ));
    assert!(matches!(got, Either::First(_)));
    assert_eq!(clock.now_ns(), 1_000_000);

    // nothing is left to wait for: block_on stops at once, without moving to 100 ms
    let mut never = SimPin::new(false);
    let stuck = catch_unwind(AssertUnwindSafe(|| block_on(never.wait_for_high())));
    assert!(stuck.is_err());
    assert_eq!(clock.now_ns(), 1_000_000);
}

// --- one clock for delays and the signal lines --------------------------------------

/// Counts system clock ticks (100 MHz)
#[derive(Default)]
struct Ticks(u64);

impl I2cPinModel for Ticks {
    fn set_inputs(&mut self, _: bool, _: bool) {}
    fn period_ps(&self) -> Option<u64> {
        Some(10_000)
    }
    fn tick(&mut self) {
        self.0 += 1;
    }
    fn sda_low(&self) -> bool {
        false
    }
}

#[test]
fn moving_time_runs_the_signal_lines() {
    let clock = SimClock::new();
    let lines = SimI2cBus::with_clock(&clock);
    let ticks = shared(Ticks::default());
    lines.attach(ticks.clone());
    let mut d = clock.delay();
    block_on(d.delay_us(2));
    assert_eq!(ticks.borrow().0, 200);
}
