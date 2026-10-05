//! Conformance tests shared by all SPI implementations.
//!
//! The same "WHO_AM_I chip" (0x0F = 0x33, 0x20 = CTRL, first byte bit7 = R, bit6 = auto-increment)
//! is built in four ways:
//!
//! 1. The Rust model (`RegisterMap`) attached at transaction level (`VirtualSpiDevice`)
//! 2. A pin-level model ported by hand from `spi_whoami.v` to Rust (`SpiWhoAmIPin`)
//! 3. The `spi_whoami.v` core modeled with Verilator
//! 4. The chip top without output enable, wrapped in a simulation wrapper
//!
//! The same tests run on all of them, and the results must match.

use embedded_hal::spi::{MODE_0, MODE_3, Mode, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

use rtl_demo::spi_whoami_pin::SpiWhoAmIPin;
use virtual_bus::bus::spi::VirtualSpiDevice;
use virtual_bus::bus::spi::sim::{SimSpiBus, SpiPinModel};
use virtual_bus::devices::register::{RegisterMap, SpiFormat};

/// The shared test run on every implementation. Returns the observed values (for comparing implementations)
fn conformance<D: SpiDevice>(dev: &mut D) -> Vec<u8> {
    let mut seen = Vec::new();

    // WHO_AM_I
    let mut b = [0x8F, 0x00];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(b[1], 0x33, "WHO_AM_I");
    seen.push(b[1]);

    // write CTRL and read it back
    dev.write(&[0x20, 0x5A]).unwrap();
    let mut b = [0xA0, 0x00];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(b[1], 0x5A, "CTRL");
    seen.push(b[1]);

    // without MS: reads the same register repeatedly
    let mut b = [0x8F, 0x00, 0x00];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(&b[1..], [0x33, 0x33], "without MS");
    seen.extend_from_slice(&b[1..]);

    // with MS: 0x0F, 0x10, ... (undefined addresses read 0)
    let mut b = [0xCF, 0x00, 0x00];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(&b[1..], [0x33, 0x00], "with MS");
    seen.extend_from_slice(&b[1..]);

    // writes to a read-only register are ignored
    dev.write(&[0x0F, 0x00]).unwrap();
    let mut b = [0x8F, 0x00];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(b[1], 0x33, "RO");
    seen.push(b[1]);

    // write + read in one transaction
    let mut r = [0u8; 1];
    dev.transaction(&mut [
        embedded_hal::spi::Operation::Write(&[0xA0]),
        embedded_hal::spi::Operation::Read(&mut r),
    ])
    .unwrap();
    assert_eq!(r[0], 0x5A, "write + read");
    seen.push(r[0]);

    seen
}

fn rust_model(mode: Mode) -> Vec<u8> {
    let map = RegisterMap::new().ro(0x0F, 0x33).rw(0x20, 0x00);
    let mut dev = VirtualSpiDevice::new(map.spi(SpiFormat::read_bit7_inc_bit6()), mode);
    conformance(&mut dev)
}

fn pin_model(model: impl SpiPinModel + 'static, mode: Mode) -> Vec<u8> {
    let bus = SimSpiBus::new();
    let cs = bus.add_device(model);
    let spi = bus.master(mode, 1_000_000).unwrap();
    let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    let seen = conformance(&mut dev);
    assert_eq!(bus.contentions(), 0);
    seen
}

#[test]
fn rust_model_mode0() {
    rust_model(MODE_0);
}

#[test]
fn hand_ported_pin_model_matches_rust_model() {
    for mode in [MODE_0, MODE_3] {
        assert_eq!(pin_model(SpiWhoAmIPin::new(), mode), rust_model(mode));
    }
}

#[test]
fn verilated_rtl_matches_rust_model() {
    use rtl_demo::spi_whoami::VerilatedSpiWhoAmI;
    for mode in [MODE_0, MODE_3] {
        assert_eq!(pin_model(VerilatedSpiWhoAmI::new(), mode), rust_model(mode));
    }
}

#[test]
fn oe_less_top_matches_rust_model() {
    use rtl_demo::spi_whoami::VerilatedSpiWhoAmITop;
    for mode in [MODE_0, MODE_3] {
        assert_eq!(
            pin_model(VerilatedSpiWhoAmITop::new(), mode),
            rust_model(mode)
        );
    }
}

#[test]
fn rtl_and_hand_port_agree_bit_for_bit_in_every_mode() {
    // the hand-ported model and the RTL should return the same values, mismatched modes included
    use embedded_hal::spi::{MODE_1, MODE_2};
    use rtl_demo::spi_whoami::VerilatedSpiWhoAmI;
    fn run(model: impl SpiPinModel + 'static, mode: Mode) -> [u8; 3] {
        let bus = SimSpiBus::new();
        let cs = bus.add_device(model);
        let spi = bus.master(mode, 1_000_000).unwrap();
        let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
        dev.write(&[0x20, 0xC3]).unwrap();
        let mut b = [0xCF, 0, 0];
        dev.transfer_in_place(&mut b).unwrap();
        b
    }
    for mode in [MODE_0, MODE_1, MODE_2, MODE_3] {
        assert_eq!(
            run(SpiWhoAmIPin::new(), mode),
            run(VerilatedSpiWhoAmI::new(), mode),
            "{mode:?}"
        );
    }
}
