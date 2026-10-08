//! Conformance tests: the same tests on a Rust model and on the RTL.
//!
//! Write the spec quickly as a Rust model, then check mechanically that the RTL behaves the same way.
//! The same "WHO_AM_I chip" (address 0x29, 0x0F = 0xA5 read-only, 0x10 = SCRATCH read-write,
//! the first written byte is the register pointer, auto-incrementing afterwards) is built in four ways:
//!
//! 1. The Rust model (`RegisterMap`) attached at transaction level (`VirtualI2cBus`)
//! 2. The same Rust model attached at pin level (`PinLevelI2cSlave` + `SimI2cBus`)
//! 3. `i2c_whoami.v` modeled with Verilator
//! 4. The chip top `i2c_whoami_top.v` (inout SDA), wrapped in a simulation wrapper
//!
//! The same tests run on all of them, and the results must match.

use embedded_hal::i2c::{Error as _, ErrorKind, I2c, NoAcknowledgeSource};

use rtl_demo::i2c_whoami::{VerilatedWhoAmI, VerilatedWhoAmITop};
use virtual_bus::VirtualI2cBus;
use virtual_bus::bus::i2c::sim::{I2cPinModel, PinLevelI2cSlave, SimI2cBus};
use virtual_bus::devices::register::{I2cFormat, RegisterMap};

const ADDR: u8 = 0x29;

/// The shared test run on every implementation. Returns the observed values (for comparing implementations)
fn conformance<I: I2c>(i2c: &mut I) -> Vec<u8> {
    let mut seen = Vec::new();
    let mut read = |i2c: &mut I, reg: u8, n: usize| -> Vec<u8> {
        let mut b = vec![0u8; n];
        i2c.write_read(ADDR, &[reg], &mut b).unwrap();
        seen.extend_from_slice(&b);
        b
    };

    // WHO_AM_I
    assert_eq!(read(i2c, 0x0F, 1), [0xA5], "WHO_AM_I");

    // write SCRATCH and read it back
    i2c.write(ADDR, &[0x10, 0x3C]).unwrap();
    assert_eq!(read(i2c, 0x10, 1), [0x3C], "SCRATCH");

    // sequential read: 0x0F, 0x10, 0x11 (undefined addresses read 0)
    assert_eq!(read(i2c, 0x0F, 3), [0xA5, 0x3C, 0x00], "sequential read");

    // writes to a read-only register are ignored
    i2c.write(ADDR, &[0x0F, 0x00]).unwrap();
    assert_eq!(read(i2c, 0x0F, 1), [0xA5], "read-only");

    // sequential write: goes into 0x10; 0x11 is not writable, so it is ignored
    i2c.write(ADDR, &[0x10, 0x5A, 0x77]).unwrap();
    assert_eq!(read(i2c, 0x10, 2), [0x5A, 0x00], "sequential write");

    // no response at another address
    assert_eq!(
        i2c.write(0x2A, &[0x00]).map_err(|e| e.kind()),
        Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    );

    seen
}

fn regs() -> RegisterMap {
    RegisterMap::new().ro(0x0F, 0xA5).rw(0x10, 0x00)
}

/// 1. Attach the Rust model at transaction level
fn rust_model() -> Vec<u8> {
    let mut bus = VirtualI2cBus::new();
    bus.attach(ADDR, regs().i2c(I2cFormat::new())).unwrap();
    conformance(&mut bus)
}

/// 2 to 4. Attach a model to the signal lines and drive it with the bit-bang master
fn pin_level(model: impl I2cPinModel + 'static) -> Vec<u8> {
    let lines = SimI2cBus::new();
    lines.attach(model);
    let mut i2c = lines.master(400_000);
    let seen = conformance(&mut i2c);
    // the bus is released even after a failed transaction
    assert!(lines.scl() && lines.sda());
    seen
}

#[test]
fn rust_model_passes() {
    rust_model();
}

#[test]
fn pin_level_rust_model_matches() {
    let model = PinLevelI2cSlave::new(ADDR, regs().i2c(I2cFormat::new()));
    assert_eq!(pin_level(model), rust_model());
}

#[test]
fn verilated_rtl_matches() {
    assert_eq!(pin_level(VerilatedWhoAmI::new()), rust_model());
}

#[test]
fn verilated_chip_top_matches() {
    assert_eq!(pin_level(VerilatedWhoAmITop::new()), rust_model());
}
