//! The simplest usage: build a model from a list of registers and attach it to virtual I2C and SPI buses.
//!
//! ```sh
//! cargo run --example register_map
//! ```

use embedded_hal::i2c::I2c;
use embedded_hal::spi::{MODE_0, Operation, SpiDevice};
use virtual_bus::VirtualI2cBus;
use virtual_bus::bus::spi::VirtualSpiDevice;
use virtual_bus::devices::register::{I2cFormat, RegisterMap, SpiFormat};

fn main() {
    // 0x0F is read-only (0xA5), 0x10 is a read-write register
    let regs = RegisterMap::new().ro(0x0F, 0xA5).rw(0x10, 0x00);

    // ---- I2C: attach at address 0x29 ----
    let mut i2c = VirtualI2cBus::new();
    i2c.attach(0x29, regs.i2c(I2cFormat::new())).unwrap();

    // Passing `i2c` to a driver here just works. Below, we read and write it directly
    let mut who = [0u8];
    i2c.write_read(0x29, &[0x0F], &mut who).unwrap();
    println!("I2C: WHO_AM_I = {:#04x}", who[0]);

    i2c.write(0x29, &[0x10, 0x5A]).unwrap();
    println!(
        "I2C: wrote 0x5a to 0x10 -> value in the model = {:#04x}",
        regs.get(0x10)
    );

    // ---- SPI: read the same map over SPI too (bit7 of the first byte = read) ----
    let mut spi = VirtualSpiDevice::new(regs.spi(SpiFormat::read_bit7_inc_bit6()), MODE_0);
    let mut buf = [0u8];
    spi.transaction(&mut [Operation::Write(&[0x80 | 0x10]), Operation::Read(&mut buf)])
        .unwrap();
    println!("SPI: 0x10 = {:#04x} (the value written over I2C)", buf[0]);

    // tests can also change values in the model and check how the driver reacts
    regs.set(0x10, 0x01);
    i2c.write_read(0x29, &[0x10], &mut who).unwrap();
    println!(
        "I2C: set to 0x01 in the model -> value read = {:#04x}",
        who[0]
    );
}
