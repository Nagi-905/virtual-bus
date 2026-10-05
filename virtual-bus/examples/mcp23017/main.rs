//! Write an IC model and drive it with `mcp23017-driver` (embedded-hal 1.0) from crates.io.
//!
//! - `model.rs`: an MCP23017 model implementing virtual-bus's [`virtual_bus::I2cSlave`]
//! - here: the driver just gets a `VirtualI2cBus`; the driver code is the same as on real hardware
//!
//! ```sh
//! cargo run --example mcp23017
//! ```

mod model;

use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use mcp23017_driver::Mcp23017 as Driver;
use model::{self as mcp23017, Mcp23017};
use virtual_bus::{LogOp, VirtualI2cBus, shared};

const ADDRESS: u8 = 0x20;

fn main() {
    let model = shared(Mcp23017::new());
    let mut bus = VirtualI2cBus::new();
    bus.attach(ADDRESS, model.clone()).unwrap();

    {
        // `&mut VirtualI2cBus` implements I2c too, so we can look at the log afterwards
        let mut device = Driver::<_, ADDRESS>::new(&mut bus);
        let (pins, _interrupts) = device.split().expect("split");

        // make A0 an output and drive it high
        let mut led = pins.a0.into_push_pull_output().unwrap();
        led.set_high().unwrap();
        println!(
            "set A0 high: pins in the model = {:#06x}",
            model.borrow().pins()
        );
        assert!(model.borrow().pin(0));
        assert!(led.is_set_high().unwrap());

        // make B3 a pulled-up input, then pull it low from outside
        let mut button = pins.b3.into_pull_up_input().unwrap();
        assert!(button.is_high().unwrap());
        model.borrow_mut().set_pin(8 + 3, false);
        assert!(button.is_low().unwrap());
        println!("pulled B3 low from outside: the driver reads low");

        led.set_low().unwrap();
        assert!(!model.borrow().pin(0));
    }

    let m = model.borrow();
    println!(
        "IODIRA={:#04x} IODIRB={:#04x} GPPUB={:#04x} OLATA={:#04x}",
        m.register(mcp23017::IODIRA),
        m.register(mcp23017::IODIRB),
        m.register(mcp23017::GPPUB),
        m.register(mcp23017::OLATA),
    );

    println!(
        "I2C transactions issued by the driver ({}):",
        bus.log().len()
    );
    for e in bus.log() {
        let ops: Vec<String> = e
            .ops
            .iter()
            .map(|op| match op {
                LogOp::Write(d) => format!("W{d:02x?}"),
                LogOp::Read(d) => format!("R{d:02x?}"),
            })
            .collect();
        println!("  {:#04x}: {}", e.address, ops.join(" "));
    }
    println!("ok");
}
