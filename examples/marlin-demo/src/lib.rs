//! virtual-bus demo: load the Verilog of `rtl-demo` with [Marlin](https://github.com/ethanuppal/marlin)
//! instead of `virtual-bus-build`, and dump waveforms.
//!
//! What changes compared with `rtl-demo`:
//!
//! - No `build.rs` and no port list. `#[verilog]` reads the ports from the Verilog and turns them
//!   into struct fields (`m.scl = 1`)
//! - Verilator runs when a model is first created, only for that model, and the result is cached
//!   under `target/marlin`. Editing one RTL file rebuilds only the models that use it, without
//!   recompiling any Rust
//! - `with_vcd` writes a VCD of every signal in the RTL, at the bus's simulated time
//!   (see [`Wave`])
//!
//! | Module | RTL | Same as |
//! |---|---|---|
//! | [`i2c_whoami`] | `i2c_whoami.v` / `sim/i2c_whoami_sim.v` / `i2c_whoami_scl.v` | `rtl_demo::i2c_whoami` |
//! | [`spi_whoami`] | `spi_whoami.v` / `sim/spi_whoami_sim.v` | `rtl_demo::spi_whoami` |
//!
//! The tests check that these models put exactly the same waveform on the lines as the
//! `rtl-demo` ones.

use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, MutexGuard};

use marlin::verilator::tracing::{OpenTrace, TraceFile, Waveform};
use marlin::verilator::{
    AsVerilatedModel, VerilatedModelConfig, VerilatorRuntime, VerilatorRuntimeOptions,
};

pub mod i2c_whoami;
pub mod spi_whoami;

/// The RTL shared with `rtl-demo`
const RTL_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../verilog/rtl");

/// Where Marlin keeps the Verilator builds (in the workspace's `target`)
const ARTIFACT_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../target/marlin");

thread_local! {
    static RUNTIMES: RefCell<HashMap<&'static [&'static str], &'static VerilatorRuntime>> =
        RefCell::default();
}

/// The Marlin runtime for these RTL files (relative to `examples/verilog/rtl`).
///
/// Models borrow their runtime, but virtual-bus wants `'static` models, so the runtime is leaked:
/// one per thread and source list, shared by every model created on that thread.
/// Verilator wants a model evaluated on the thread that created it, which this also keeps
fn runtime(sources: &'static [&'static str]) -> &'static VerilatorRuntime {
    RUNTIMES.with_borrow_mut(|map| {
        *map.entry(sources).or_insert_with(|| {
            let paths: Vec<PathBuf> = sources.iter().map(|s| Path::new(RTL_DIR).join(s)).collect();
            let rt = VerilatorRuntime::new2(
                ARTIFACT_DIR,
                &paths,
                &[] as &[&Path],
                [],
                VerilatorRuntimeOptions::default(),
            )
            .expect("failed to set up Marlin (Verilator 5.025 or later is required)");
            Box::leak(Box::new(rt))
        })
    })
}

/// Creates model `M` from `sources`. With `vcd`, the model is built with tracing and its waveform
/// goes to that file
fn create<M>(sources: &'static [&'static str], vcd: Option<&Path>) -> (M, Wave)
where
    M: AsVerilatedModel<'static> + OpenTrace<'static, Trace = TraceFile<'static>>,
{
    let config = VerilatedModelConfig::default()
        .verilator_optimization(3)
        .enable_tracing(vcd.map(|_| Waveform::Vcd));
    let mut model: M = runtime(sources)
        .create_model(&config)
        .unwrap_or_else(|e| panic!("failed to build {}: {e}", M::name()));
    let wave = match vcd {
        Some(path) => {
            let guard = TraceGuard::acquire();
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir).unwrap();
            }
            Wave::new(Some((model.open_trace(path), guard)))
        }
        None => Wave::new(None),
    };
    (model, wave)
}

static TRACING: Mutex<()> = Mutex::new(());

thread_local! {
    static TRACING_ON_THIS_THREAD: Cell<bool> = const { Cell::new(false) };
}

/// Only one model may write a waveform at a time in a process.
///
/// Marlin creates every model in Verilator's default context, and two models tracing at once
/// (for example in tests running in parallel) lose parts of each other's waveforms.
/// So a traced model holds this lock until it is dropped; other threads wait for it
struct TraceGuard(#[allow(dead_code)] MutexGuard<'static, ()>);

impl TraceGuard {
    fn acquire() -> Self {
        assert!(
            !TRACING_ON_THIS_THREAD.get(),
            "only one model can write a waveform at a time; drop the other traced model first"
        );
        let guard = TRACING.lock().unwrap_or_else(|e| e.into_inner());
        TRACING_ON_THIS_THREAD.set(true);
        Self(guard)
    }
}

impl Drop for TraceGuard {
    fn drop(&mut self) {
        TRACING_ON_THIS_THREAD.set(false);
    }
}

/// Writes a model's waveform at the bus's simulated time (ps).
///
/// The bus often evaluates a model several times at one moment (it redelivers the inputs until the
/// lines settle), but Verilator ignores a second dump at the same time. So the values for a moment
/// are written when the time moves on, once the model has settled.
/// Adapters call [`Self::set_time`] from `set_time_ps` and [`Self::evaluated`] after each eval.
pub struct Wave {
    /// The open VCD, and the lock held while it is open (dropped after the file is closed)
    file: Option<(TraceFile<'static>, TraceGuard)>,
    now: u64,
    /// Evaluated at `now` and not yet dumped
    dirty: bool,
    /// The time of the last dump
    last: Option<u64>,
}

impl Wave {
    fn new(file: Option<(TraceFile<'static>, TraceGuard)>) -> Self {
        Self {
            file,
            now: 0,
            // the reset done when the model is created shows at time 0
            dirty: true,
            last: None,
        }
    }

    /// Whether a waveform is being written
    pub fn is_on(&self) -> bool {
        self.file.is_some()
    }

    fn dump(&mut self, t: u64) {
        if self.last.is_some_and(|last| t <= last) {
            return;
        }
        if let Some((file, _)) = &mut self.file {
            file.dump(t);
            self.last = Some(t);
        }
    }

    /// The bus moved the time to `t`
    pub fn set_time(&mut self, t: u64) {
        if t != self.now {
            if self.dirty {
                self.dump(self.now);
                self.dirty = false;
            }
            self.now = t;
        }
    }

    /// The model was evaluated at the current time
    pub fn evaluated(&mut self) {
        self.dirty = true;
    }

    /// For models on a system clock: the bus only calls `tick` at the rising edge, so the low half
    /// of the clock is written half a period earlier (if nothing was written after that point)
    pub fn clock_low(&mut self, period_ps: u64) {
        self.dump(self.now.saturating_sub(period_ps / 2));
    }
}

impl Drop for Wave {
    fn drop(&mut self) {
        if self.dirty {
            self.dump(self.now);
        }
    }
}
