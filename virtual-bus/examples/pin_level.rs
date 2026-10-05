//! Pin-level usage: drive the signal lines with bit-bang masters and advance simulated time.
//!
//! The signal lines (`SimI2cBus` / `SimSpiBus`) and masters are used the same way when attaching RTL.
//!
//! ```sh
//! cargo run --example pin_level
//! ```

use embedded_hal::i2c::I2c;
use embedded_hal::spi::{MODE_0, MODE_1, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use virtual_bus::bus::i2c::sim::{PinLevelI2cSlave, SimI2cBus};
use virtual_bus::bus::spi::sim::{PinLevelSpiSlave, SimSpiBus};
use virtual_bus::devices::register::{I2cFormat, RegisterMap, SpiFormat};

fn main() {
    let regs = RegisterMap::new().ro(0x0F, 0xA5).rw(0x10, 0x00);

    // ---- I2C: attach a model to the lines and drive it with a 400 kHz master ----
    let lines = SimI2cBus::new();
    lines.attach(PinLevelI2cSlave::new(0x29, regs.i2c(I2cFormat::new())));
    let mut i2c = lines.master(400_000);

    let mut who = [0u8];
    i2c.write_read(0x29, &[0x0F], &mut who).unwrap();
    println!(
        "I2C: WHO_AM_I = {:#04x} (took {} ns)",
        who[0],
        lines.now_ns()
    );

    // ---- SPI: leave CS to embedded-hal-bus's ExclusiveDevice ----
    let lines = SimSpiBus::new();
    let cs = lines.add_device(PinLevelSpiSlave::new(
        regs.spi(SpiFormat::read_bit7_inc_bit6()),
        MODE_0,
    ));
    let mut spi =
        ExclusiveDevice::new(lines.master(MODE_0, 1_000_000).unwrap(), cs, lines.delay()).unwrap();

    let mut buf = [0x80 | 0x0F, 0];
    spi.transfer_in_place(&mut buf).unwrap();
    println!("SPI (mode 0): WHO_AM_I = {:#04x}", buf[1]);

    // ---- with mismatched SPI modes, data gets garbled just like on real hardware ----
    let lines = SimSpiBus::new();
    let cs = lines.add_device(PinLevelSpiSlave::new(
        regs.spi(SpiFormat::read_bit7_inc_bit6()),
        MODE_0,
    ));
    let mut spi =
        ExclusiveDevice::new(lines.master(MODE_1, 1_000_000).unwrap(), cs, lines.delay()).unwrap();

    let mut buf = [0x80 | 0x0F, 0];
    spi.transfer_in_place(&mut buf).unwrap();
    println!(
        "SPI (master only in mode 1): WHO_AM_I = {:#04x} (not 0xa5)",
        buf[1]
    );
}
