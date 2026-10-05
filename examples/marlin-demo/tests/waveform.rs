//! The VCDs written by `with_vcd` follow the bus's simulated time.

use std::collections::HashMap;

use embedded_hal::i2c::I2c;
use embedded_hal::spi::{MODE_0, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

use marlin_demo::i2c_whoami::{MarlinWhoAmI, MarlinWhoAmIScl};
use marlin_demo::spi_whoami::MarlinSpiWhoAmI;
use virtual_bus::bus::i2c::sim::SimI2cBus;
use virtual_bus::bus::spi::sim::SimSpiBus;

/// A small VCD reader: the changes of each top-level signal as (time, value)
struct Vcd {
    timescale: String,
    changes: HashMap<String, Vec<(u64, String)>>,
}

impl Vcd {
    fn read(path: &str) -> Self {
        let text = std::fs::read_to_string(path).unwrap();
        let mut timescale = String::new();
        let mut names: HashMap<String, String> = HashMap::new(); // id → name (first scope wins)
        let mut changes: HashMap<String, Vec<(u64, String)>> = HashMap::new();
        let mut now = 0;
        let mut words = text.split_whitespace().peekable();
        while let Some(w) = words.next() {
            match w {
                "$timescale" => timescale = words.next().unwrap().to_string(),
                "$var" => {
                    let _kind = words.next();
                    let _width = words.next();
                    let id = words.next().unwrap().to_string();
                    let name = words.next().unwrap().to_string();
                    names.entry(id).or_insert(name);
                }
                _ if w.starts_with('#') => now = w[1..].parse().unwrap(),
                _ if w.starts_with('b') => {
                    let id = words.next().unwrap();
                    if let Some(n) = names.get(id) {
                        changes
                            .entry(n.clone())
                            .or_default()
                            .push((now, w[1..].to_string()));
                    }
                }
                _ if w.len() > 1 && matches!(&w[..1], "0" | "1" | "x" | "z") => {
                    if let Some(n) = names.get(&w[1..]) {
                        changes
                            .entry(n.clone())
                            .or_default()
                            .push((now, w[..1].to_string()));
                    }
                }
                _ => {}
            }
        }
        Self { timescale, changes }
    }

    /// The times at which `name` changed (the first entry is its initial value)
    fn edges(&self, name: &str) -> Vec<u64> {
        self.changes[name].iter().skip(1).map(|c| c.0).collect()
    }
}

fn path(name: &str) -> String {
    format!("{}/{name}", env!("CARGO_TARGET_TMPDIR"))
}

#[test]
fn scl_edges_in_the_vcd_are_the_bus_edges() {
    let file = path("i2c_whoami_scl.vcd");
    let bus = SimI2cBus::new();
    bus.attach(MarlinWhoAmIScl::with_vcd(&file));
    let mut i2c = bus.master(400_000);
    bus.run_ns(1_000);
    bus.enable_trace();
    i2c.write(0x29, &[0x10, 0x96]).unwrap();
    let mut b = [0u8; 2];
    i2c.write_read(0x29, &[0x0F], &mut b).unwrap();
    assert_eq!(b, [0xA5, 0x96]);
    let trace = bus.trace();
    drop((i2c, bus)); // closes the VCD

    let vcd = Vcd::read(&file);
    assert_eq!(vcd.timescale, "1ps");
    let bus_scl: Vec<u64> = trace
        .windows(2)
        .filter(|w| w[0].scl != w[1].scl)
        .map(|w| w[1].time_ps)
        .collect();
    let first = trace[0].time_ps; // the trace starts at the first change
    let vcd_scl: Vec<u64> = vcd
        .edges("scl")
        .into_iter()
        .filter(|&t| t > first)
        .collect();
    assert!(bus_scl.len() > 50);
    assert_eq!(vcd_scl, bus_scl);
    // the slave's own output toggles too (ACKs and read data)
    assert!(vcd.edges("sda_low").len() > 10);
}

#[test]
fn system_clock_shows_both_halves() {
    let file = path("i2c_whoami.vcd");
    let bus = SimI2cBus::new();
    bus.attach(MarlinWhoAmI::with_vcd(&file));
    bus.run_ns(1_000);
    drop(bus);

    let vcd = Vcd::read(&file);
    let clk = &vcd.changes["clk"];
    // 50 MHz for 1 µs: 50 rising edges, each with a low half 10 ns before it
    // (a VCD lists only changes, so the first low half, same as clk's initial 0, is not listed)
    let rises: Vec<u64> = clk.iter().filter(|c| c.1 == "1").map(|c| c.0).collect();
    let falls: Vec<u64> = clk
        .iter()
        .skip(1)
        .filter(|c| c.1 == "0")
        .map(|c| c.0)
        .collect();
    assert_eq!(rises.len(), 50);
    assert_eq!(rises[0], 20_000);
    assert_eq!(rises[1] - rises[0], 20_000);
    assert_eq!(rises[1] - falls[0], 10_000);
}

#[test]
#[should_panic(expected = "only one model can write a waveform at a time")]
fn two_traced_models_on_one_thread_are_refused() {
    let _a = MarlinWhoAmIScl::with_vcd(path("refused_a.vcd"));
    let _b = MarlinWhoAmIScl::with_vcd(path("refused_b.vcd"));
}

#[test]
fn spi_vcd_has_internal_state() {
    let file = path("spi_whoami.vcd");
    let bus = SimSpiBus::new();
    let cs = bus.add_device(MarlinSpiWhoAmI::with_vcd(&file));
    let spi = bus.master(MODE_0, 1_000_000).unwrap();
    let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    let mut b = [0x8F, 0];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(b[1], 0x33);
    drop((dev, bus));

    let vcd = Vcd::read(&file);
    // 16 SCK rising edges at 1 MHz
    let sck_rises = vcd.changes["sck"].iter().filter(|c| c.1 == "1").count();
    assert_eq!(sck_rises, 16);
    // signals inside the module are there as well, not just the ports
    assert!(vcd.changes.len() > 6, "{:?}", vcd.changes.keys());
}
