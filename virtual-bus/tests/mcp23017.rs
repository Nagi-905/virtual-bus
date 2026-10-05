//! Tests for the example MCP23017 model (`examples/mcp23017/model.rs`).
//! Checks register reads and writes, and that `mcp23017-driver` from crates.io runs unchanged.

#[path = "../examples/mcp23017/model.rs"]
mod model;

use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use embedded_hal::i2c::I2c;
use mcp23017_driver::Mcp23017 as Driver;
use model::*;
use virtual_bus::bus::i2c::sim::{PinLevelI2cSlave, SimI2cBus};
use virtual_bus::{Shared, VirtualI2cBus, shared};

fn setup() -> (VirtualI2cBus, Shared<Mcp23017>) {
    let m = shared(Mcp23017::new());
    let mut bus = VirtualI2cBus::new();
    bus.attach(0x20, m.clone()).unwrap();
    (bus, m)
}

fn read(bus: &mut VirtualI2cBus, reg: u8) -> u8 {
    let mut b = [0];
    bus.write_read(0x20, &[reg], &mut b).unwrap();
    b[0]
}

#[test]
fn power_on_state() {
    let (mut bus, _) = setup();
    assert_eq!(read(&mut bus, IODIRA), 0xFF);
    assert_eq!(read(&mut bus, IODIRB), 0xFF);
    assert_eq!(read(&mut bus, GPIOA), 0x00);
}

#[test]
fn outputs_follow_olat() {
    let (mut bus, m) = setup();
    bus.write(0x20, &[IODIRA, 0x0F]).unwrap(); // A4..A7 as outputs
    bus.write(0x20, &[GPIOA, 0xA0]).unwrap();
    assert_eq!(read(&mut bus, OLATA), 0xA0);
    assert_eq!(m.borrow().pins() & 0xFF, 0xA0);
    assert!(m.borrow().pin(7));
    assert!(!m.borrow().pin(6));
}

#[test]
fn inputs_with_pullup_external_and_polarity() {
    let (mut bus, m) = setup();
    bus.write(0x20, &[GPPUB, 0x03]).unwrap();
    m.borrow_mut().set_pin(8, false); // B0 pulled low from outside
    m.borrow_mut().set_pin(10, true); // B2 driven high from outside
    assert_eq!(read(&mut bus, GPIOB), 0b0000_0110);
    bus.write(0x20, &[IPOLB, 0x04]).unwrap();
    assert_eq!(read(&mut bus, GPIOB), 0b0000_0010);
    m.borrow_mut().release_pin(8);
    assert_eq!(read(&mut bus, GPIOB), 0b0000_0011);
}

#[test]
fn sequential_access_auto_increments_and_wraps() {
    let (mut bus, _) = setup();
    bus.write(0x20, &[IPOLA, 0x11, 0x22, 0x33]).unwrap();
    let mut b = [0u8; 3];
    bus.write_read(0x20, &[IPOLA], &mut b).unwrap();
    assert_eq!(b, [0x11, 0x22, 0x33]);
    let mut b = [0u8; 2];
    bus.write_read(0x20, &[OLATB], &mut b).unwrap();
    assert_eq!(b, [0x00, 0xFF]); // IODIRA comes after OLATB
}

#[test]
fn ipol_does_not_affect_outputs() {
    let (mut bus, _) = setup();
    bus.write(0x20, &[IODIRA, 0x00, 0x00]).unwrap();
    bus.write(0x20, &[IPOLA, 0xFF]).unwrap();
    bus.write(0x20, &[GPIOA, 0x5A]).unwrap();
    assert_eq!(read(&mut bus, GPIOA), 0x5A);
}

/// Drives the model through the driver, recording the pin states and the values the driver reads
fn driver_scenario<I: I2c>(i2c: I, model: &Shared<Mcp23017>) -> Vec<u32> {
    let mut seen = Vec::new();
    let mut device = Driver::<_, 0x20>::new(i2c);
    let (p, _interrupts) = device.split().unwrap();
    let mut led = p.a0.into_push_pull_output().unwrap();
    led.set_high().unwrap();
    seen.push(u32::from(model.borrow().pins()));
    seen.push(u32::from(led.is_set_high().unwrap()));
    let mut button = p.b3.into_pull_up_input().unwrap();
    seen.push(u32::from(button.is_high().unwrap()));
    model.borrow_mut().set_pin(8 + 3, false);
    seen.push(u32::from(button.is_low().unwrap()));
    led.set_low().unwrap();
    seen.push(u32::from(model.borrow().pins()));
    seen
}

#[test]
fn crates_io_driver_runs_unchanged_on_both_paths() {
    // transaction level
    let (mut bus, m) = setup();
    let fast = driver_scenario(&mut bus, &m);

    // pin level (BitBangI2c → signal lines → PinLevelI2cSlave → model)
    let m2 = shared(Mcp23017::new());
    let lines = SimI2cBus::new();
    lines.attach(PinLevelI2cSlave::new(0x20, m2.clone()));
    let slow = driver_scenario(lines.master(400_000), &m2);

    assert_eq!(fast, slow);
    assert_eq!(fast, [0x0001, 1, 1, 1, 0x0000]);
    assert!(lines.now_ns() > 0);
}
