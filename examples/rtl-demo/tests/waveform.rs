//! VCDs written by models built with `.trace()` follow the bus's simulated time.

use std::collections::HashMap;
use std::io::ErrorKind;
use std::thread;

use embedded_hal::spi::{MODE_0, SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

use rtl_demo::bindings::{i2c_whoami, spi_counter};
use rtl_demo::spi_counter::SpiCounter;
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

    /// The times at which `name` went to `value`
    fn times(&self, name: &str, value: &str) -> Vec<u64> {
        self.changes[name]
            .iter()
            .filter(|c| c.1 == value)
            .map(|c| c.0)
            .collect()
    }

    /// The last value of `name`
    fn last(&self, name: &str) -> &str {
        &self.changes[name].last().unwrap().1
    }
}

fn path(name: &str) -> String {
    format!("{}/{name}", env!("CARGO_TARGET_TMPDIR"))
}

/// Sets PRESCALE, enables the counter, waits and reads COUNT on a 1 MHz mode 0 bus.
/// Returns the bytes read and the simulated time at the end
fn counter_traffic(dut: SpiCounter, prescale: u8) -> (u8, u8, u64) {
    let bus = SimSpiBus::new();
    let cs = bus.add_device(dut);
    let spi = bus.master(MODE_0, 1_000_000).unwrap();
    let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    dev.write(&[SpiCounter::REG_PRESCALE, prescale]).unwrap();
    dev.write(&[SpiCounter::REG_CTRL, SpiCounter::CTRL_EN])
        .unwrap();
    bus.run_ns(10_000);
    let mut b = [0x80 | SpiCounter::REG_PRESCALE, 0, 0];
    dev.transfer_in_place(&mut b).unwrap();
    (b[1], b[2], bus.now_ps())
    // the bus, and with it the model and its VCD, are dropped here
}

#[test]
fn sck_edges_in_the_vcd_follow_the_bus_time() {
    let file = path("spi_counter_frame.vcd");
    let bus = SimSpiBus::new();
    let cs = bus.add_device(SpiCounter::with_vcd(&file).unwrap());
    let spi = bus.master(MODE_0, 1_000_000).unwrap();
    let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    let mut b = [0x80 | SpiCounter::REG_ID, 0];
    dev.transfer_in_place(&mut b).unwrap();
    assert_eq!(b[1], SpiCounter::ID);
    drop((dev, bus));

    let vcd = Vcd::read(&file);
    assert_eq!(vcd.timescale, "1ps");
    // mode 0 at 1 MHz: 16 rising SCK edges, the first half a period after CS falls, then one per µs
    let selected = vcd.times("cs_n", "0")[0];
    let expected: Vec<u64> = (0..16)
        .map(|k| selected + 500_000 + k * 1_000_000)
        .collect();
    assert_eq!(vcd.times("sck", "1"), expected);
    assert_eq!(
        *vcd.times("cs_n", "1").last().unwrap(),
        selected + 16_000_000
    );
    // the chip drives the ID on MISO in the second byte: 0x5C = 0101_1100 rises twice
    let id_byte = selected + 8_000_000..selected + 16_000_000;
    let rises = vcd.times("miso", "1");
    assert_eq!(rises.iter().filter(|t| id_byte.contains(t)).count(), 2);
}

#[test]
fn tracing_does_not_change_the_results() {
    assert_eq!(
        counter_traffic(SpiCounter::with_vcd(path("same.vcd")).unwrap(), 9),
        counter_traffic(SpiCounter::new(), 9)
    );
}

#[test]
fn system_clock_shows_both_halves() {
    let file = path("clock.vcd");
    let bus = SimSpiBus::new();
    let cs = bus.add_device(SpiCounter::with_vcd(&file).unwrap());
    bus.run_ns(1_000);
    // the CS pin holds the bus too: drop both so the model closes its VCD
    drop((cs, bus));

    let vcd = Vcd::read(&file);
    // 50 MHz for 1 µs: 50 rising edges, each with a low half 10 ns before it
    // (a VCD lists only changes, so the first low half, same as clk's initial 0, is not listed)
    let rises = vcd.times("clk", "1");
    let falls: Vec<u64> = vcd.times("clk", "0").into_iter().skip(1).collect();
    assert_eq!(rises.len(), 50);
    assert_eq!(rises[0], 20_000);
    assert_eq!(rises[1] - rises[0], 20_000);
    assert_eq!(rises[1] - falls[0], 10_000);
}

#[test]
fn back_to_back_frames_show_cs_high_in_the_vcd() {
    let file = path("spi_counter.vcd");
    let bus = SimSpiBus::new();
    let cs = bus.add_device(SpiCounter::with_vcd(&file).unwrap());
    let spi = bus.master(MODE_0, 1_000_000).unwrap();
    let mut dev = ExclusiveDevice::new(spi, cs, bus.delay()).unwrap();
    // PRESCALE = 0, then EN, then read COUNT: three frames with no wait in between
    dev.write(&[SpiCounter::REG_PRESCALE, 0]).unwrap();
    dev.write(&[SpiCounter::REG_CTRL, SpiCounter::CTRL_EN])
        .unwrap();
    let mut b = [0x80 | SpiCounter::REG_COUNT, 0];
    dev.transfer_in_place(&mut b).unwrap();
    drop((dev, bus));

    let vcd = Vcd::read(&file);
    // cs_n: initial 1, then 0 / 1 for each of the three frames
    let cs_n: Vec<&str> = vcd.changes["cs_n"].iter().map(|c| c.1.as_str()).collect();
    assert_eq!(cs_n, ["1", "0", "1", "0", "1", "0", "1"]);
    // each high between frames lasts one SCK period (1 µs)
    let t: Vec<u64> = vcd.changes["cs_n"].iter().map(|c| c.0).collect();
    assert_eq!(t[3] - t[2], 1_000_000);
    assert_eq!(t[5] - t[4], 1_000_000);
    // signals inside the module are there as well: the counter, enabled in the second frame,
    // moves while CS is high before the third
    let counting = vcd.changes["count"]
        .iter()
        .filter(|c| (t[4]..t[5]).contains(&c.0))
        .count();
    assert_eq!(counting, 50, "one count per 20 ns system clock for 1 µs");
}

#[test]
fn models_trace_in_parallel_without_interfering() {
    // every model has its own Verilator context, so traces on several threads at once are complete
    let results: Vec<_> = thread::scope(|s| {
        let handles: Vec<_> = (0..4u8)
            .map(|i| {
                s.spawn(move || {
                    let file = path(&format!("parallel_{i}.vcd"));
                    let read = counter_traffic(SpiCounter::with_vcd(&file).unwrap(), 10 + i);
                    (file, read, i)
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    for (file, (prescale, _count, _), i) in results {
        assert_eq!(prescale, 10 + i);
        let vcd = Vcd::read(&file);
        // three frames of 2, 2 and 3 bytes
        assert_eq!(vcd.times("sck", "1").len(), 56, "{file}");
        assert_eq!(vcd.last("prescale"), format!("{:08b}", 10 + i), "{file}");
    }
}

#[test]
fn models_without_trace_refuse_a_vcd() {
    // i2c_whoami is built without .trace() in build.rs
    let mut raw = RawModel::new(&i2c_whoami::VTABLE);
    assert!(!raw.can_trace());
    let e = raw.open_vcd(path("refused.vcd")).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::Unsupported);
    assert!(e.to_string().contains("add .trace()"), "{e}");
}

#[test]
fn a_second_vcd_on_one_model_is_refused() {
    let mut raw = RawModel::new(&spi_counter::VTABLE);
    raw.open_vcd(path("first.vcd")).unwrap();
    let e = raw.open_vcd(path("second.vcd")).unwrap_err();
    assert_eq!(e.kind(), ErrorKind::AlreadyExists);
}
