//! VCDs written by models built with `.trace()` follow the bus's simulated time.

use std::collections::HashMap;
use std::io::ErrorKind;
use std::thread;

use embedded_hal::i2c::I2c;
use embedded_hal::spi::{MODE_0, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

use rtl_demo::bindings::i2c_whoami_scl;
use rtl_demo::i2c_whoami::VerilatedWhoAmI;
use rtl_demo::spi_whoami::VerilatedSpiWhoAmI;
use virtual_bus::bus::i2c::sim::{I2cPinModel, LineEvent, SimI2cBus};
use virtual_bus::bus::spi::sim::SimSpiBus;
use virtual_bus::verilated::RawModel;

/// A small VCD reader: the changes of each signal as (time, value), by name (first scope wins)
struct Vcd {
    timescale: String,
    changes: HashMap<String, Vec<(u64, String)>>,
}

impl Vcd {
    fn read(path: &str) -> Self {
        let text = std::fs::read_to_string(path).unwrap();
        let mut timescale = String::new();
        let mut names: HashMap<String, String> = HashMap::new(); // id → name
        let mut changes: HashMap<String, Vec<(u64, String)>> = HashMap::new();
        let mut now = 0;
        let mut words = text.split_whitespace();
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

/// Runs I2C traffic with the bus trace on. Returns the line changes and the bytes read
fn i2c_traffic(model: impl I2cPinModel + 'static, value: u8) -> (Vec<LineEvent>, [u8; 2]) {
    let bus = SimI2cBus::new();
    bus.attach(model);
    let mut i2c = bus.master(400_000);
    bus.run_ns(1_000);
    bus.enable_trace();
    i2c.write(0x29, &[0x10, value]).unwrap();
    let mut b = [0u8; 2];
    i2c.write_read(0x29, &[0x0F], &mut b).unwrap();
    (bus.trace(), b)
    // the bus, and with it the model and its VCD, are dropped here
}

/// SCL edges in a trace (the first entry is where recording started)
fn scl_edges(trace: &[LineEvent]) -> Vec<u64> {
    trace
        .windows(2)
        .filter(|w| w[0].scl != w[1].scl)
        .map(|w| w[1].time_ps)
        .collect()
}

#[test]
fn scl_edges_in_the_vcd_are_the_bus_edges() {
    let file = path("i2c_whoami.vcd");
    let (trace, read) = i2c_traffic(VerilatedWhoAmI::with_vcd(&file).unwrap(), 0x96);
    assert_eq!(read, [0xA5, 0x96]);

    let vcd = Vcd::read(&file);
    assert_eq!(vcd.timescale, "1ps");
    let bus = scl_edges(&trace);
    let first = trace[0].time_ps;
    let in_vcd: Vec<u64> = vcd
        .edges("scl")
        .into_iter()
        .filter(|&t| t > first)
        .collect();
    assert!(bus.len() > 50);
    assert_eq!(in_vcd, bus);
    // the slave's own output toggles too (ACKs and read data)
    assert!(vcd.edges("sda_low").len() > 10);
}

#[test]
fn tracing_does_not_change_the_lines() {
    assert_eq!(
        i2c_traffic(
            VerilatedWhoAmI::with_vcd(path("same_lines.vcd")).unwrap(),
            0x5A
        ),
        i2c_traffic(VerilatedWhoAmI::new(), 0x5A)
    );
}

#[test]
fn system_clock_shows_both_halves() {
    let file = path("clock.vcd");
    let bus = SimI2cBus::new();
    bus.attach(VerilatedWhoAmI::with_vcd(&file).unwrap());
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
fn spi_vcd_has_internal_state() {
    let file = path("spi_whoami.vcd");
    let bus = SimSpiBus::new();
    let cs = bus.add_device(VerilatedSpiWhoAmI::with_vcd(&file).unwrap());
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

#[test]
fn models_trace_in_parallel_without_interfering() {
    // every model has its own Verilator context, so traces on several threads at once are complete
    let results: Vec<_> = thread::scope(|s| {
        let handles: Vec<_> = (0..4u8)
            .map(|i| {
                s.spawn(move || {
                    let file = path(&format!("parallel_{i}.vcd"));
                    let (trace, read) =
                        i2c_traffic(VerilatedWhoAmI::with_vcd(&file).unwrap(), 0x10 + i);
                    (file, trace, read, i)
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for (file, trace, read, i) in results {
        assert_eq!(read, [0xA5, 0x10 + i]);
        let vcd = Vcd::read(&file);
        let first = trace[0].time_ps;
        let in_vcd: Vec<u64> = vcd
            .edges("scl")
            .into_iter()
            .filter(|&t| t > first)
            .collect();
        assert_eq!(in_vcd, scl_edges(&trace), "{file}");
    }
}

#[test]
fn models_without_trace_refuse_a_vcd() {
    let mut raw = RawModel::new(&i2c_whoami_scl::VTABLE);
    assert!(!raw.can_trace());
    let e = raw.open_vcd(path("refused.vcd")).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::Unsupported);
    assert!(e.to_string().contains("add .trace()"), "{e}");
}

#[test]
fn a_second_vcd_on_one_model_is_refused() {
    let mut raw = RawModel::new(&rtl_demo::bindings::i2c_whoami::VTABLE);
    raw.open_vcd(path("first.vcd")).unwrap();
    let e = raw.open_vcd(path("second.vcd")).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::AlreadyExists);
}
