//! Drive the spi_counter RTL through an embedded-hal `SpiDevice`, and write a VCD to look at.
//!
//! ```sh
//! cargo run -p rtl-demo --example spi_counter
//! gtkwave target/spi_counter.vcd
//! ```

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{MODE_0, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;
use rtl_demo::spi_counter::SpiCounter;
use virtual_bus::bus::spi::sim::SimSpiBus;
use virtual_bus::shared;

const VCD: &str = "target/spi_counter.vcd";

fn main() {
    // the signal lines, and the RTL attached to them. The model is shared so that
    // `irq`, which is not an SPI pin, can be read from here
    let bus = SimSpiBus::new();
    let mut delay = bus.delay();
    let dut = shared(SpiCounter::with_vcd(VCD).expect("cannot create the VCD"));
    let cs = bus.add_device(dut.clone());

    // a 1 MHz mode 0 master and a device on it, as a driver would get them on real hardware
    let spi = bus.master(MODE_0, 1_000_000).unwrap();
    let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    let read = |dev: &mut ExclusiveDevice<_, _, _>, reg: u8| {
        let mut b = [0x80 | reg, 0];
        dev.transfer_in_place(&mut b).unwrap();
        b[1]
    };

    println!("ID       = 0x{:02X}", read(&mut dev, SpiCounter::REG_ID));

    dev.write(&[SpiCounter::REG_PRESCALE, 49]).unwrap(); // +1 every 50 clk = 1 µs
    dev.write(&[SpiCounter::REG_COMPARE, 40]).unwrap(); // MATCH at COUNT = 40
    dev.write(&[
        SpiCounter::REG_CTRL,
        SpiCounter::CTRL_EN | SpiCounter::CTRL_IRQ_EN,
    ])
    .unwrap();
    let started = bus.now_ns();
    println!("started: PRESCALE = 49 (1 µs per count), COMPARE = 40");

    // the counter keeps running while we wait and while each read is on the bus (about 17 µs).
    // COUNT is the value latched after the command byte, about 8 µs before the read ends,
    // so it trails the time printed here by about 8 counts
    for _ in 0..4 {
        delay.delay_us(10);
        let count = read(&mut dev, SpiCounter::REG_COUNT);
        println!(
            "{:>3} µs after start: COUNT = {count:>2}  irq = {}",
            (bus.now_ns() - started) / 1000,
            dut.borrow().irq()
        );
    }

    // clear MATCH (write 1 to STATUS) with the counter stopped
    dev.write(&[SpiCounter::REG_CTRL, SpiCounter::CTRL_IRQ_EN])
        .unwrap();
    dev.write(&[SpiCounter::REG_STATUS, 0x01]).unwrap();
    println!("cleared STATUS: irq = {}", dut.borrow().irq());

    // the VCD is closed when the last handle to the model (the bus, its delay, the device) is dropped
    drop((dev, delay, bus, dut));
    println!("wrote {VCD}");
}
