//! `mcp23017-driver` from crates.io, unchanged, on the example MCP23017 model.
//! The same scenario on both paths (transaction level and signal lines) gives the same result

#[path = "../examples/mcp23017/model.rs"]
mod model;

use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use embedded_hal::i2c::I2c;
use mcp23017_driver::Mcp23017 as Driver;
use model::Mcp23017;
use virtual_bus::bus::i2c::sim::{PinLevelI2cSlave, SimI2cBus};
use virtual_bus::{Shared, VirtualI2cBus, shared};

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
    let m = shared(Mcp23017::new());
    let mut bus = VirtualI2cBus::new();
    bus.attach(0x20, m.clone()).unwrap();
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
