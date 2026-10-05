//! Runs one I2C read against the RTL and writes its waveform.
//!
//! ```sh
//! cargo run -p marlin-demo --example i2c_wave
//! gtkwave target/i2c_whoami.vcd
//! ```

use embedded_hal::i2c::I2c;
use marlin_demo::i2c_whoami::MarlinWhoAmI;
use virtual_bus::bus::i2c::sim::SimI2cBus;

fn main() {
    let file = concat!(env!("CARGO_MANIFEST_DIR"), "/../../target/i2c_whoami.vcd");
    let bus = SimI2cBus::new();
    bus.attach(MarlinWhoAmI::with_vcd(file));
    let mut i2c = bus.master(400_000);

    let mut who = [0u8];
    i2c.write_read(MarlinWhoAmI::ADDRESS, &[0x0F], &mut who)
        .unwrap();
    println!("WHO_AM_I = {:#04x}", who[0]);

    drop((i2c, bus)); // closes the VCD
    println!(
        "waveform: {}",
        std::path::Path::new(file).canonicalize().unwrap().display()
    );
}
