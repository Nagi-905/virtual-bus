//! The Marlin models must behave exactly like the `rtl-demo` (virtual-bus-build) models of the same RTL:
//! the same line changes to the picosecond on I2C, and the same bytes in every mode on SPI.

use embedded_hal::i2c::{Error as _, ErrorKind, I2c, NoAcknowledgeSource};
use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3, Mode, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

use marlin_demo::i2c_whoami::{MarlinWhoAmI, MarlinWhoAmIScl, MarlinWhoAmITop};
use marlin_demo::spi_whoami::{MarlinSpiWhoAmI, MarlinSpiWhoAmITop};
use rtl_demo::i2c_whoami::{VerilatedWhoAmI, VerilatedWhoAmIScl, VerilatedWhoAmITop};
use rtl_demo::spi_whoami::{VerilatedSpiWhoAmI, VerilatedSpiWhoAmITop};
use virtual_bus::bus::i2c::sim::{I2cPinModel, LineEvent, SimI2cBus};
use virtual_bus::bus::spi::sim::{SimSpiBus, SpiPinModel};
use virtual_bus::shared;

const ADDR: u8 = 0x29;

/// Runs the same I2C traffic and returns the line changes and the bytes read
fn i2c_run(model: impl I2cPinModel + 'static) -> (Vec<LineEvent>, Vec<u8>) {
    let bus = SimI2cBus::new();
    bus.attach(model);
    let mut i2c = bus.master(400_000);
    bus.enable_trace();
    let mut seen = Vec::new();

    let mut b = [0u8; 3];
    i2c.write(ADDR, &[0x10, 0x3C]).unwrap();
    i2c.write_read(ADDR, &[0x0F], &mut b).unwrap();
    seen.extend_from_slice(&b);
    i2c.write(ADDR, &[0x0F, 0x00]).unwrap(); // read-only
    i2c.write(ADDR, &[0x10, 0x5A, 0x77]).unwrap();
    i2c.write_read(ADDR, &[0x0F], &mut b).unwrap();
    seen.extend_from_slice(&b);
    assert_eq!(
        i2c.write(0x2A, &[0x00]).map_err(|e| e.kind()),
        Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    );
    assert!(bus.scl() && bus.sda());
    (bus.trace(), seen)
}

#[test]
fn i2c_core_matches() {
    let marlin = i2c_run(MarlinWhoAmI::new());
    assert_eq!(marlin.1, [0xA5, 0x3C, 0x00, 0xA5, 0x5A, 0x00]);
    assert_eq!(marlin, i2c_run(VerilatedWhoAmI::new()));
}

#[test]
fn i2c_top_matches() {
    assert_eq!(
        i2c_run(MarlinWhoAmITop::new()),
        i2c_run(VerilatedWhoAmITop::new())
    );
}

#[test]
fn i2c_scl_only_matches() {
    assert_eq!(
        i2c_run(MarlinWhoAmIScl::new()),
        i2c_run(VerilatedWhoAmIScl::new())
    );
}

#[test]
fn tracing_does_not_change_the_lines() {
    let dir = env!("CARGO_TARGET_TMPDIR");
    assert_eq!(
        i2c_run(MarlinWhoAmI::with_vcd(format!("{dir}/matches_core.vcd"))),
        i2c_run(MarlinWhoAmI::new())
    );
}

#[test]
fn i2c_top_power_cycle_resets_registers() {
    let dut = shared(MarlinWhoAmITop::new_unpowered());
    let bus = SimI2cBus::new();
    bus.attach(dut.clone());
    let mut i2c = bus.master(400_000);
    let mut b = [0u8];

    assert_eq!(
        i2c.write(ADDR, &[0x0F]),
        Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address))
    );
    dut.borrow_mut().set_vdd(true);
    i2c.write(ADDR, &[0x10, 0x5C]).unwrap();
    i2c.write_read(ADDR, &[0x10], &mut b).unwrap();
    assert_eq!(b[0], 0x5C);

    dut.borrow_mut().set_vdd(false);
    bus.run_ns(1_000);
    dut.borrow_mut().set_vdd(true);
    bus.run_ns(1_000);
    i2c.write_read(ADDR, &[0x10], &mut b).unwrap();
    assert_eq!(b[0], 0x00);
}

/// Runs the same SPI traffic and returns the bytes read and the MISO contention count
fn spi_run(model: impl SpiPinModel + 'static, mode: Mode) -> (Vec<u8>, usize) {
    let bus = SimSpiBus::new();
    let cs = bus.add_device(model);
    let spi = bus.master(mode, 1_000_000).unwrap();
    let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    let mut seen = Vec::new();
    for frame in [
        &[0x8F, 0, 0][..],
        &[0x20, 0xC3],
        &[0xA0, 0],
        &[0xCF, 0, 0],
        &[0x0F, 0],
    ] {
        let mut b = frame.to_vec();
        dev.transfer_in_place(&mut b).unwrap();
        seen.extend_from_slice(&b);
    }
    (seen, bus.contentions())
}

#[test]
fn spi_core_matches_in_every_mode() {
    for mode in [MODE_0, MODE_1, MODE_2, MODE_3] {
        assert_eq!(
            spi_run(MarlinSpiWhoAmI::new(), mode),
            spi_run(VerilatedSpiWhoAmI::new(), mode),
            "{mode:?}"
        );
    }
    assert_eq!(spi_run(MarlinSpiWhoAmI::new(), MODE_0).0[1], 0x33);
}

#[test]
fn spi_top_matches_in_every_mode() {
    for mode in [MODE_0, MODE_1, MODE_2, MODE_3] {
        assert_eq!(
            spi_run(MarlinSpiWhoAmITop::new(), mode),
            spi_run(VerilatedSpiWhoAmITop::new(), mode),
            "{mode:?}"
        );
    }
}

#[test]
fn spi_top_unpowered_is_silent() {
    let dut = shared(MarlinSpiWhoAmITop::new_unpowered());
    let bus = SimSpiBus::new();
    let cs = bus.add_device(dut.clone());
    let spi = bus.master(MODE_0, 2_000_000).unwrap();
    let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    let mut b = [0x8F, 0];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(b[1], 0xFF); // Hi-Z, pulled up
    dut.borrow_mut().set_vdd(true);
    let mut b = [0x8F, 0];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(b[1], 0x33);
}
