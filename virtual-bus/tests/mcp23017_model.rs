//! The example MCP23017 model (`examples/mcp23017/model.rs`) on its own: registers and pins,
//! read and written straight over the bus, no driver

#[path = "../examples/mcp23017/model.rs"]
mod model;

use embedded_hal::i2c::I2c;
use model::*;
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
