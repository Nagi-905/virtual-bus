//! Runs `lis3dh-async` (crates.io, unchanged) on a LIS3DH register map, over I2C and SPI.
//!
//! Values from the datasheet (ST DocID17530 Rev 2):
//! - Defaults: WHO_AM_I 33h, CTRL_REG0 10h, CTRL_REG1 07h, others 00h (Table 21)
//! - Bits: CTRL_REG1 = ODR[3:0] LPen Zen Yen Xen (8.8), CTRL_REG4 = BDU BLE FS[1:0] HR ST[1:0] SIM
//!   (8.11), TEMP_CFG_REG bit 7 = ADC_EN
//! - Output: HR, ±2 g = 1 mg/digit, 12 bits left-justified, LSB first (Table 4, 8.16-8.18)
//! - Auto-increment: I2C bit 7 (6.1.1). SPI: bit 7 = read, bit 6 = increment, mode 3 (6.2)

use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
use embedded_hal::spi::MODE_3;
use lis3dh_async::{Error, Lis3dh, SlaveAddr};
use virtual_bus::bus::spi::VirtualSpiDevice;
use virtual_bus::devices::register::{I2cFormat, RegisterMap, SpiFormat};
use virtual_bus::{Fault, VirtualI2cBus, block_on};

const CTRL_REG1: u8 = 0x20;
const CTRL_REG4: u8 = 0x23;
const TEMP_CFG_REG: u8 = 0x1F;
const OUT_X_L: u8 = 0x28;

fn lis3dh() -> RegisterMap {
    let mut m = RegisterMap::new()
        .ro(0x0F, 0x33)
        .rw(0x1E, 0x10)
        .rw(TEMP_CFG_REG, 0x00)
        .rw(CTRL_REG1, 0x07);
    for a in 0x21..=0x25 {
        m = m.rw(a, 0x00);
    }
    for a in 0x27..=0x2D {
        m = m.ro(a, 0x00);
    }
    m
}

/// Sets what the sensor reads, in mg per axis (HR, ±2 g)
fn expose(m: &RegisterMap, mg: [i16; 3]) {
    for (i, v) in mg.into_iter().enumerate() {
        let [lo, hi] = (v << 4).to_le_bytes();
        m.set(OUT_X_L + 2 * i as u8, lo);
        m.set(OUT_X_L + 2 * i as u8 + 1, hi);
    }
}

fn assert_default_config(m: &RegisterMap) {
    // 400 Hz, X/Y/Z on
    assert_eq!(m.get(CTRL_REG1), 0x77);
    // BDU, high resolution, ±2 g
    assert_eq!(m.get(CTRL_REG4), 0x88);
    assert_eq!(m.get(TEMP_CFG_REG) & 0x80, 0x80, "ADC_EN");
}

#[test]
fn configures_and_reads_over_i2c() {
    let m = lis3dh();
    let mut bus = VirtualI2cBus::new();
    bus.attach(0x18, m.i2c(I2cFormat::inc_bit7())).unwrap();

    let mut accel = block_on(Lis3dh::new_i2c(&mut bus, SlaveAddr::Default)).unwrap();
    assert_default_config(&m);

    expose(&m, [-500, 250, 1000]);
    let g = block_on(accel.accel_norm()).unwrap();
    assert_eq!((g.x, g.y, g.z), (-0.5, 0.25, 1.0));
}

#[test]
fn configures_and_reads_over_spi() {
    let m = lis3dh();
    let mut dev = VirtualSpiDevice::new(m.spi(SpiFormat::read_bit7_inc_bit6()), MODE_3);

    let mut accel = block_on(Lis3dh::new_spi(&mut dev)).unwrap();
    assert_default_config(&m);

    expose(&m, [12, -34, 1000]);
    let raw = block_on(accel.accel_raw()).unwrap();
    assert_eq!((raw.x >> 4, raw.y >> 4, raw.z >> 4), (12, -34, 1000));
}

#[test]
fn several_calls_in_one_block() {
    let m = lis3dh();
    let mut bus = VirtualI2cBus::new();
    bus.attach(0x18, m.i2c(I2cFormat::inc_bit7())).unwrap();
    expose(&m, [0, 0, 1000]);

    let (id, g) = block_on(async {
        let mut accel = Lis3dh::new_i2c(&mut bus, SlaveAddr::Default).await?;
        let id = accel.get_device_id().await?;
        let g = accel.accel_norm().await?;
        Ok::<_, Error<ErrorKind>>((id, g))
    })
    .unwrap();
    assert_eq!((id, g.z), (0x33, 1.0));
}

#[test]
fn bus_errors_reach_the_async_driver() {
    // nobody at 0x19
    let mut bus = VirtualI2cBus::new();
    bus.attach(0x18, lis3dh().i2c(I2cFormat::inc_bit7()))
        .unwrap();
    assert!(matches!(
        block_on(Lis3dh::new_i2c(&mut bus, SlaveAddr::Alternate)),
        Err(Error::Bus(ErrorKind::NoAcknowledge(
            NoAcknowledgeSource::Address
        )))
    ));

    // NACK the next transaction (the axis read)
    bus.inject_fault(Fault::AddressNack);
    let mut accel = block_on(Lis3dh::new_i2c_without_config(&mut bus, SlaveAddr::Default));
    assert!(matches!(
        block_on(accel.accel_raw()),
        Err(Error::Bus(ErrorKind::NoAcknowledge(_)))
    ));
}

// --- faults reach the driver, and it works again afterwards -------------------

#[test]
fn i2c_faults_reach_the_driver_and_it_recovers() {
    let m = lis3dh();
    let mut bus = VirtualI2cBus::new();
    bus.attach(0x18, m.i2c(I2cFormat::inc_bit7())).unwrap();
    for fault in [
        Fault::BusError,
        Fault::ArbitrationLoss,
        Fault::DataNack { index: 0 },
    ] {
        bus.inject_fault(fault);
        let mut accel = block_on(Lis3dh::new_i2c_without_config(&mut bus, SlaveAddr::Default));
        assert!(
            matches!(block_on(accel.get_device_id()), Err(Error::Bus(_))),
            "{fault:?}"
        );
        // the next transaction is clean
        assert_eq!(block_on(accel.get_device_id()).unwrap(), 0x33);
    }
}

#[test]
fn spi_faults_reach_the_driver_and_it_recovers() {
    use embedded_hal::spi::ErrorKind as SpiErrorKind;
    use virtual_bus::bus::spi::{SpiError, SpiFault};

    let m = lis3dh();
    let mut dev = VirtualSpiDevice::new(m.spi(SpiFormat::read_bit7_inc_bit6()), MODE_3);

    // an error from the bus
    dev.inject_fault(SpiFault::Error(SpiErrorKind::Other));
    assert!(matches!(
        block_on(Lis3dh::new_spi(&mut dev)),
        Err(Error::Bus(SpiError::Injected(SpiErrorKind::Other)))
    ));

    // MISO stuck high: WHO_AM_I reads FFh and the driver refuses the chip
    dev.inject_fault(SpiFault::MisoStuck(0xFF));
    assert!(matches!(
        block_on(Lis3dh::new_spi(&mut dev)),
        Err(Error::WrongAddress)
    ));

    // clean again: the driver configures the chip
    block_on(Lis3dh::new_spi(&mut dev)).unwrap();
    assert_eq!(m.get(CTRL_REG1), 0x77);

    // a flipped bit in OUT_X_L (byte 1 after the command): no error, a wrong value
    expose(&m, [0, 0, 1000]);
    dev.inject_fault(SpiFault::FlipBits {
        index: 1,
        mask: 0x10,
    });
    let mut accel = block_on(Lis3dh::new_spi_without_config(&mut dev));
    let raw = block_on(accel.accel_raw()).unwrap();
    assert_eq!(raw.x >> 4, 1, "1 mg off, and the driver cannot tell");
    let raw = block_on(accel.accel_raw()).unwrap();
    assert_eq!((raw.x >> 4, raw.z >> 4), (0, 1000));
}

// --- the same driver gives the same results on both SPI paths -----------------

async fn read_all<S>(spi: S) -> (u8, [i16; 3])
where
    S: embedded_hal_async::spi::SpiDevice,
    S::Error: core::fmt::Debug,
{
    let mut accel = Lis3dh::new_spi(spi).await.unwrap();
    let id = accel.get_device_id().await.unwrap();
    let raw = accel.accel_raw().await.unwrap();
    (id, [raw.x, raw.y, raw.z])
}

#[test]
fn the_same_driver_reads_the_same_on_both_spi_paths() {
    use embedded_hal_bus::spi::ExclusiveDevice;
    use virtual_bus::bus::spi::sim::{PinLevelSpiSlave, SimSpiBus};

    let format = SpiFormat::read_bit7_inc_bit6;

    // transaction level
    let a = lis3dh();
    expose(&a, [-500, 250, 1000]);
    let txn = block_on(read_all(VirtualSpiDevice::new(a.spi(format()), MODE_3)));

    // signal level: the same register map behind a pin-level adapter and a 1 MHz bit-bang master
    let b = lis3dh();
    expose(&b, [-500, 250, 1000]);
    let lines = SimSpiBus::new();
    let cs = lines.add_device(PinLevelSpiSlave::new(b.spi(format()), MODE_3));
    let master = lines.master(MODE_3, 1_000_000).unwrap();
    let dev = ExclusiveDevice::new(master, cs, lines.delay()).unwrap();
    let pin = block_on(read_all(dev));

    assert_eq!(txn, pin);
    assert_eq!(txn.0, 0x33);
    for r in [TEMP_CFG_REG, CTRL_REG1, CTRL_REG4] {
        assert_eq!(a.get(r), b.get(r), "register {r:#04x}");
    }
}
