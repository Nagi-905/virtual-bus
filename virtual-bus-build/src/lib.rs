//! A helper called from `build.rs`. Converts Verilog to C++ with Verilator, generates a C ABI glue layer
//! and the Rust bindings, and links them.
//!
//! No hand-written C++ and no port list: the ports are read from what Verilator generates, so the
//! bindings always match the RTL (including widths set by parameters such as `-GWIDTH=12`).
//!
//! ```ignore
//! // build.rs
//! fn main() {
//!     virtual_bus_build::Verilated::new()
//!         .rtl_dir("rtl")
//!         .model(
//!             virtual_bus_build::Model::new("spi_counter", "spi_counter")
//!                 .source("spi_counter.v")
//!                 .flag("-Wall"),
//!         )
//!         .compile();
//! }
//! ```
//!
//! ```ignore
//! // src/lib.rs (include the generated bindings)
//! pub mod bindings {
//!     include!(concat!(env!("OUT_DIR"), "/verilated_models.rs"));
//! }
//!
//! let mut m = bindings::spi_counter::Model::new();
//! m.set_cs_n(true); // one setter per input
//! m.eval();
//! let driving = m.miso_oe(); // one getter per output
//! ```
//!
//! Each model's module holds:
//!
//! - `Model`: the model with typed methods. `set_<input>(v)` for each input and `<output>()` for each
//!   output; 1-bit ports are `bool`, wider ones `u8` / `u16` / `u32` / `u64`. Setters do not
//!   evaluate; call `eval` after setting the inputs
//! - `VTABLE` and one pin-number constant per port (`CS_N`, `MISO`, ...), for reading and writing
//!   through `virtual_bus::verilated::RawModel` directly (`Model::raw_mut`)
//!
//! Supported top-level ports are inputs and outputs of 1 to 64 bits. `inout` ports and wider ports
//! stop the build with a message; wrap the top in a module that splits them.
//!
//! Requirements: Verilator 5.x (`verilator` on PATH, or `VERILATOR_ROOT`) and a C++17 compiler.
//! The using crate must depend on `virtual-bus` (the bindings refer to `virtual_bus::verilated`).
//!
//! # Rebuilds
//!
//! Each model is its own static library, cached in `OUT_DIR`. A model is rebuilt only when its
//! settings or one of the files Verilator read for it (its sources and anything they `include`,
//! taken from Verilator's dependency file) changed in content. The Verilator runtime is compiled
//! once per Verilator version and compiler settings. Whatever needs building is built in parallel.

use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;

/// One RTL design to model with Verilator
#[derive(Debug, Clone)]
pub struct Model {
    name: String,
    top: String,
    sources: Vec<PathBuf>,
    flags: Vec<String>,
    trace: bool,
}

impl Model {
    /// `name` is the name of the generated Rust module and the prefix of the C symbols (lowercase letters,
    /// digits and `_`). `top` is the name of the Verilog top module
    pub fn new(name: &str, top: &str) -> Self {
        assert!(
            !name.is_empty()
                && name
                    .chars()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_'),
            "model names may only contain lowercase letters, digits and _: {name}"
        );
        Self {
            name: name.to_string(),
            top: top.to_string(),
            sources: Vec::new(),
            flags: Vec::new(),
            trace: false,
        }
    }

    /// A Verilog source (relative to [`Verilated::rtl_dir`], or an absolute path)
    pub fn source(mut self, path: impl AsRef<Path>) -> Self {
        self.sources.push(path.as_ref().to_path_buf());
        self
    }

    /// Extra flags passed to verilator (`-Wall`, `-Wno-fatal`, `-GWIDTH=12` and so on)
    pub fn flag(mut self, flag: &str) -> Self {
        self.flags.push(flag.to_string());
        self
    }

    /// Builds the model with tracing, so that it can write a VCD of all its signals
    /// (`Model::open_vcd`). Costs some build time, and a little simulation speed even when no VCD
    /// is open
    pub fn trace(mut self) -> Self {
        self.trace = true;
        self
    }
}

/// A top-level port, as Verilator declares it in the model's header
#[derive(Debug, Clone, PartialEq, Eq)]
struct Port {
    name: String,
    msb: u32,
    lsb: u32,
    input: bool,
}

impl Port {
    fn width(&self) -> u32 {
        self.msb - self.lsb + 1
    }

    /// The Rust type of the port's value
    fn rust_type(&self) -> &'static str {
        match self.width() {
            1 => "bool",
            2..=8 => "u8",
            9..=16 => "u16",
            17..=32 => "u32",
            _ => "u64",
        }
    }

    /// How the port is written in Verilog, for docs: `scl`, `data[7:0]`
    fn verilog(&self) -> String {
        if self.width() == 1 && self.lsb == 0 {
            self.name.clone()
        } else {
            format!("{}[{}:{}]", self.name, self.msb, self.lsb)
        }
    }
}

/// Builds several models together
#[derive(Debug, Default)]
pub struct Verilated {
    rtl_dir: Option<PathBuf>,
    models: Vec<Model>,
}

impl Verilated {
    pub fn new() -> Self {
        Self::default()
    }

    /// Base directory of the Verilog sources (the crate root by default). Relative paths are relative to the crate root
    pub fn rtl_dir(mut self, dir: impl AsRef<Path>) -> Self {
        self.rtl_dir = Some(dir.as_ref().to_path_buf());
        self
    }

    pub fn model(mut self, model: Model) -> Self {
        self.models.push(model);
        self
    }

    /// Runs verilator, compiles the output together with the glue and links it.
    /// The bindings are written to `$OUT_DIR/verilated_models.rs`.
    ///
    /// Models whose settings and input files have not changed are reused from the previous build
    pub fn compile(self) {
        let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
        let rtl = match self.rtl_dir {
            Some(d) if d.is_absolute() => d,
            Some(d) => manifest.join(d),
            None => manifest,
        };
        let out = PathBuf::from(env::var("OUT_DIR").unwrap());
        let tc = Toolchain::detect(&verilator_root());
        let pkg = env::var("CARGO_PKG_NAME").unwrap().replace('-', "_");
        for (i, m) in self.models.iter().enumerate() {
            assert!(
                !self.models[..i].iter().any(|o| o.name == m.name),
                "duplicate model name: {}",
                m.name
            );
        }

        let (runtime, models) = thread::scope(|s| {
            let runtime = s.spawn(|| build_runtime(&tc, &out, &pkg));
            let models: Vec<_> = self
                .models
                .iter()
                .map(|m| s.spawn(|| build_model(m, &rtl, &tc, &out, &pkg)))
                .collect();
            (
                join(runtime),
                models.into_iter().map(join).collect::<Vec<_>>(),
            )
        });

        // the models first: they use the runtime
        for (lib, _) in &models {
            println!("cargo::rustc-link-search=native={}", lib.dir.display());
            println!("cargo::rustc-link-lib=static={}", lib.name);
        }
        println!("cargo::rustc-link-search=native={}", runtime.dir.display());
        println!("cargo::rustc-link-lib=static={}", runtime.name);
        if let Some(stdlib) = tc.cxx_stdlib() {
            println!("cargo::rustc-link-lib={stdlib}");
        }

        let mut rs = String::from("// Generated by virtual-bus-build. Do not edit\n");
        for (m, (_, ports)) in self.models.iter().zip(&models) {
            rs.push_str(&bindings_rs(&m.name, &m.top, ports, m.trace));
        }
        fs::write(out.join("verilated_models.rs"), rs).unwrap();
    }
}

/// Bump when the way the C++ is compiled changes, so that cached builds are thrown away
/// (changes to the glue are noticed without this)
const CACHE_REV: u32 = 2;

/// What the compiled C++ depends on besides the RTL. A change rebuilds everything
struct Toolchain {
    include: PathBuf,
    target: String,
    key: String,
}

impl Toolchain {
    fn detect(root: &Path) -> Self {
        let version = Command::new("verilator")
            .arg("--version")
            .output()
            .expect("verilator not found (Verilator 5.x is required)");
        let target = env::var("TARGET").unwrap();
        let mut key = format!(
            "rev {CACHE_REV}\n{}\nroot {}\ntarget {target}\n",
            String::from_utf8_lossy(&version.stdout).trim(),
            root.display()
        );
        let t = target.replace('-', "_");
        for var in [
            "CXX".to_string(),
            "CXXFLAGS".to_string(),
            "CXXSTDLIB".to_string(),
            format!("CXX_{target}"),
            format!("CXXFLAGS_{target}"),
            format!("CXX_{t}"),
            format!("CXXFLAGS_{t}"),
        ] {
            println!("cargo::rerun-if-env-changed={var}");
            writeln!(key, "{var}={}", env::var(&var).unwrap_or_default()).unwrap();
        }
        Self {
            include: root.join("include"),
            target,
            key,
        }
    }

    /// A C++ build with the settings shared by the runtime and the models, writing into `dir`.
    /// Link flags are printed by [`Verilated::compile`], not by `cc`
    fn cc(&self, dir: &Path, trace: bool) -> cc::Build {
        let trace = if trace { "1" } else { "0" };
        let mut cc = cc::Build::new();
        cc.cpp(true)
            .std("c++17")
            .opt_level(2) // optimize the simulation even in debug builds
            .warnings(false)
            .include(&self.include)
            .include(self.include.join("vltstd"))
            .define("VM_COVERAGE", "0")
            .define("VM_SC", "0")
            .define("VM_TRACE", trace)
            .define("VM_TRACE_VCD", trace)
            .out_dir(dir)
            .cargo_metadata(false);
        cc
    }

    /// The C++ standard library to link (what `cc` would choose)
    fn cxx_stdlib(&self) -> Option<String> {
        if let Ok(lib) = env::var("CXXSTDLIB") {
            return (!lib.is_empty()).then_some(lib);
        }
        let t = &self.target;
        if t.contains("msvc") {
            None
        } else if t.contains("apple") || t.contains("freebsd") || t.contains("openbsd") {
            Some("c++".into())
        } else {
            Some("stdc++".into())
        }
    }
}

/// A static library built by this crate
struct Lib {
    dir: PathBuf,
    name: String,
}

impl Lib {
    fn exists(&self) -> bool {
        self.dir.join(format!("lib{}.a", self.name)).exists()
            || self.dir.join(format!("{}.lib", self.name)).exists()
    }
}

fn join<T>(handle: thread::ScopedJoinHandle<'_, T>) -> T {
    handle
        .join()
        .unwrap_or_else(|e| std::panic::resume_unwind(e))
}

fn reset_dir(dir: &Path) {
    if dir.exists() {
        fs::remove_dir_all(dir).unwrap();
    }
    fs::create_dir_all(dir).unwrap();
}

/// The Verilator runtime (`verilated.cpp` and friends, and the VCD writer for traced models),
/// compiled once per toolchain
fn build_runtime(tc: &Toolchain, out: &Path, pkg: &str) -> Lib {
    let dir = out.join("verilated_runtime");
    let lib = Lib {
        dir: dir.clone(),
        name: format!("vb_{pkg}_verilated_runtime"),
    };
    let settings = hash_str(&tc.key);
    if lib.exists() && Stamp::read(&dir).is_some_and(|s| s.is_current(settings)) {
        return lib;
    }
    reset_dir(&dir);
    tc.cc(&dir, false)
        .file(tc.include.join("verilated.cpp"))
        .file(tc.include.join("verilated_threads.cpp"))
        .file(tc.include.join("verilated_vcd_c.cpp"))
        .compile(&lib.name);
    Stamp::new(settings, Vec::new()).write(&dir);
    lib
}

/// One model: verilator, the glue and its C++, unless nothing it depends on changed.
/// Returns the library and the model's ports
fn build_model(m: &Model, rtl: &Path, tc: &Toolchain, out: &Path, pkg: &str) -> (Lib, Vec<Port>) {
    let dir = out.join("verilated").join(&m.name);
    let lib = Lib {
        dir: dir.clone(),
        name: format!("vb_{pkg}_{}", m.name),
    };
    let sources: Vec<PathBuf> = m
        .sources
        .iter()
        .map(|s| {
            if s.is_absolute() {
                s.clone()
            } else {
                rtl.join(s)
            }
        })
        .collect();
    for p in &sources {
        assert!(p.exists(), "RTL not found: {}", p.display());
        println!("cargo::rerun-if-changed={}", p.display());
    }
    let settings = hash_str(&format!("{}{m:?}\n{sources:?}\n", tc.key));
    let header = dir.join(format!("Vvb_{}.h", m.name));
    let glue_path = dir.join(format!("vb_{}_glue.cpp", m.name));

    if lib.exists() {
        if let Some(stamp) = Stamp::read(&dir).filter(|s| s.is_current(settings)) {
            // the glue is generated from the ports; if this crate now generates different glue,
            // fall through and rebuild
            let ports = fs::read_to_string(&header)
                .ok()
                .and_then(|h| parse_ports(&h).ok());
            if let Some(ports) = ports {
                let glue = fs::read_to_string(&glue_path).unwrap_or_default();
                if glue == glue_cpp(&m.name, &ports, m.trace) {
                    stamp.watch();
                    return (lib, ports);
                }
            }
        }
    }

    reset_dir(&dir);
    let mut cmd = Command::new("verilator");
    cmd.arg("--cc")
        .arg("--prefix")
        .arg(format!("Vvb_{}", m.name))
        .arg("--top-module")
        .arg(&m.top)
        .arg("-Mdir")
        .arg(&dir)
        .arg("-O3")
        .args(m.trace.then_some("--trace"))
        .args(&m.flags)
        .args(&sources);
    let st = cmd.status().expect("failed to start verilator");
    assert!(st.success(), "verilator failed: {}", m.name);

    let ports = parse_ports(&fs::read_to_string(&header).expect("verilator wrote no header"))
        .unwrap_or_else(|e| panic!("model {} (top `{}`): {e}", m.name, m.top));
    fs::write(&glue_path, glue_cpp(&m.name, &ports, m.trace)).unwrap();
    let mut cc = tc.cc(&dir, m.trace);
    cc.include(&dir).files(cpp_files(&dir)).compile(&lib.name);

    let depfile = fs::read_to_string(dir.join(format!("Vvb_{}__ver.d", m.name)))
        .expect("verilator wrote no dependency file");
    let stamp = Stamp::new(settings, verilator_inputs(&depfile));
    stamp.write(&dir);
    stamp.watch();
    (lib, ports)
}

/// The top-level ports declared in a Verilator model header (`VL_IN8(&scl,0,0);` and so on)
fn parse_ports(header: &str) -> Result<Vec<Port>, String> {
    let mut ports = Vec::new();
    for line in header.lines() {
        let line = line.trim();
        let Some((mac, rest)) = line.split_once('(') else {
            continue;
        };
        let input = match mac {
            "VL_IN8" | "VL_IN16" | "VL_IN" | "VL_IN64" => true,
            "VL_OUT8" | "VL_OUT16" | "VL_OUT" | "VL_OUT64" => false,
            "VL_INW" | "VL_OUTW" | "VL_INOUTW" | "VL_INOUT8" | "VL_INOUT16" | "VL_INOUT"
            | "VL_INOUT64" => {
                let name = rest.split(',').next().unwrap_or("").trim_start_matches('&');
                return Err(if mac.ends_with('W') {
                    format!(
                        "port `{name}` is wider than 64 bits, which is not supported; \
                         wrap the top in a module that splits it"
                    )
                } else {
                    format!(
                        "port `{name}` is an inout, which is not supported; wrap the top in a \
                         module that splits it into an input and an output"
                    )
                });
            }
            _ => continue,
        };
        let args: Vec<&str> = rest
            .split(')')
            .next()
            .unwrap_or("")
            .split(',')
            .map(str::trim)
            .collect();
        let [name, msb, lsb] = args[..] else {
            return Err(format!("unexpected port declaration: {line}"));
        };
        let num = |s: &str| {
            s.parse::<u32>()
                .map_err(|_| format!("unexpected port declaration: {line}"))
        };
        ports.push(Port {
            name: name.trim_start_matches('&').to_string(),
            msb: num(msb)?,
            lsb: num(lsb)?,
            input,
        });
    }
    Ok(ports)
}

/// The input files listed in Verilator's dependency file (`<outputs> : <inputs>`)
fn verilator_inputs(depfile: &str) -> Vec<PathBuf> {
    let inputs = depfile.split_once(" : ").map_or("", |(_, i)| i);
    let mut v: Vec<PathBuf> = inputs.split_whitespace().map(PathBuf::from).collect();
    v.sort();
    v.dedup();
    v
}

/// What a cached build was made from: a hash of its settings, and the input files with a hash of
/// their contents. Kept as `vb_stamp` next to the build
struct Stamp {
    settings: u64,
    inputs: Vec<(PathBuf, Option<u64>)>,
}

impl Stamp {
    const FILE: &str = "vb_stamp";

    fn new(settings: u64, inputs: Vec<PathBuf>) -> Self {
        let inputs = inputs
            .into_iter()
            .map(|p| {
                let h = hash_input(&p);
                (p, h)
            })
            .collect();
        Self { settings, inputs }
    }

    fn read(dir: &Path) -> Option<Self> {
        let text = fs::read_to_string(dir.join(Self::FILE)).ok()?;
        let mut lines = text.lines();
        let settings = u64::from_str_radix(lines.next()?.strip_prefix("settings ")?, 16).ok()?;
        let mut inputs = Vec::new();
        for line in lines {
            let (h, path) = line.split_once(' ')?;
            let h = if h == "-" {
                None
            } else {
                Some(u64::from_str_radix(h, 16).ok()?)
            };
            inputs.push((PathBuf::from(path), h));
        }
        Some(Self { settings, inputs })
    }

    fn write(&self, dir: &Path) {
        let mut s = format!("settings {:016x}\n", self.settings);
        for (path, h) in &self.inputs {
            match h {
                Some(h) => writeln!(s, "{h:016x} {}", path.display()).unwrap(),
                None => writeln!(s, "- {}", path.display()).unwrap(),
            }
        }
        fs::write(dir.join(Self::FILE), s).unwrap();
    }

    /// The settings are the same and no input file changed
    fn is_current(&self, settings: u64) -> bool {
        self.settings == settings
            && self
                .inputs
                .iter()
                .all(|(p, h)| p.exists() && hash_input(p) == *h)
    }

    /// Asks cargo to rerun the build script when an input file changes
    fn watch(&self) {
        for (path, _) in &self.inputs {
            println!("cargo::rerun-if-changed={}", path.display());
        }
    }
}

/// The content hash of an input file. The Verilator binary is skipped (its version is part of the
/// settings, and hashing it is slow)
fn hash_input(path: &Path) -> Option<u64> {
    let skip = path
        .file_name()
        .is_some_and(|n| n.to_string_lossy().starts_with("verilator_bin"));
    if skip {
        return None;
    }
    fs::read(path).ok().map(|b| fnv1a(&b))
}

fn hash_str(s: &str) -> u64 {
    fnv1a(s.as_bytes())
}

/// FNV-1a (64 bit): stable across Rust versions, unlike `DefaultHasher`
fn fnv1a(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| {
        (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

fn verilator_root() -> PathBuf {
    println!("cargo::rerun-if-env-changed=VERILATOR_ROOT");
    if let Ok(root) = env::var("VERILATOR_ROOT") {
        return PathBuf::from(root);
    }
    let out = Command::new("verilator")
        .args(["--getenv", "VERILATOR_ROOT"])
        .output()
        .expect("verilator not found (Verilator 5.x is required)");
    assert!(
        out.status.success(),
        "verilator --getenv VERILATOR_ROOT failed"
    );
    PathBuf::from(String::from_utf8(out.stdout).unwrap().trim())
}

fn cpp_files(dir: &Path) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|e| e == "cpp"))
        .collect();
    v.sort();
    v
}

/// The C ABI glue (new / free / eval / set / get, and trace_open / trace_dump / trace_close when
/// the model is traced)
fn glue_cpp(name: &str, ports: &[Port], trace: bool) -> String {
    let class = format!("Vvb_{name}");
    let n = name;
    let mut s = String::new();
    writeln!(s, "// Generated by virtual-bus-build. Do not edit").unwrap();
    writeln!(s, "#include <cstdint>").unwrap();
    writeln!(s, "#include \"verilated.h\"").unwrap();
    if trace {
        writeln!(s, "#include \"verilated_vcd_c.h\"").unwrap();
    }
    writeln!(s, "#include \"{class}.h\"").unwrap();
    writeln!(s, "namespace {{").unwrap();
    writeln!(s, "struct Inst {{").unwrap();
    writeln!(s, "  VerilatedContext ctx;").unwrap();
    if trace {
        // must be on before the model is constructed; members are initialized in this order
        writeln!(s, "  bool trace_ever_on = (ctx.traceEverOn(true), true);").unwrap();
    }
    writeln!(s, "  {class} top;").unwrap();
    if trace {
        writeln!(s, "  VerilatedVcdC* vcd = nullptr;").unwrap();
    }
    writeln!(s, "  Inst() : ctx(), top(&ctx, \"top\") {{}}").unwrap();
    writeln!(s, "}};").unwrap();
    writeln!(s, "}}").unwrap();
    writeln!(
        s,
        "extern \"C\" void* vb_{n}_new() {{ return new Inst(); }}"
    )
    .unwrap();
    if trace {
        writeln!(
            s,
            "extern \"C\" void vb_{n}_trace_close(void* p) {{ Inst* i = static_cast<Inst*>(p); if (i->vcd) {{ i->vcd->close(); delete i->vcd; i->vcd = nullptr; }} }}"
        )
        .unwrap();
        writeln!(
            s,
            "extern \"C\" bool vb_{n}_trace_open(void* p, const char* path) {{"
        )
        .unwrap();
        writeln!(s, "  Inst* i = static_cast<Inst*>(p);").unwrap();
        writeln!(s, "  if (i->vcd) return false;").unwrap();
        writeln!(s, "  i->vcd = new VerilatedVcdC;").unwrap();
        writeln!(s, "  i->top.trace(i->vcd, 99);").unwrap();
        writeln!(s, "  i->vcd->open(path);").unwrap();
        writeln!(
            s,
            "  if (!i->vcd->isOpen()) {{ delete i->vcd; i->vcd = nullptr; return false; }}"
        )
        .unwrap();
        writeln!(s, "  return true;").unwrap();
        writeln!(s, "}}").unwrap();
        writeln!(
            s,
            "extern \"C\" void vb_{n}_trace_dump(void* p, uint64_t t) {{ Inst* i = static_cast<Inst*>(p); if (i->vcd) i->vcd->dump(t); }}"
        )
        .unwrap();
        writeln!(
            s,
            "extern \"C\" void vb_{n}_free(void* p) {{ vb_{n}_trace_close(p); Inst* i = static_cast<Inst*>(p); i->top.final(); delete i; }}"
        )
        .unwrap();
    } else {
        writeln!(
            s,
            "extern \"C\" void vb_{n}_free(void* p) {{ Inst* i = static_cast<Inst*>(p); i->top.final(); delete i; }}"
        )
        .unwrap();
    }
    writeln!(
        s,
        "extern \"C\" void vb_{n}_eval(void* p) {{ static_cast<Inst*>(p)->top.eval(); }}"
    )
    .unwrap();
    writeln!(
        s,
        "extern \"C\" void vb_{n}_set(void* p, uint32_t pin, uint64_t v) {{"
    )
    .unwrap();
    writeln!(s, "  {class}& t = static_cast<Inst*>(p)->top;").unwrap();
    writeln!(s, "  switch (pin) {{").unwrap();
    for (i, port) in ports.iter().enumerate() {
        if port.input {
            writeln!(s, "  case {i}: t.{} = v; break;", port.name).unwrap();
        }
    }
    writeln!(s, "  default: break;").unwrap();
    writeln!(s, "  }}").unwrap();
    writeln!(s, "}}").unwrap();
    writeln!(
        s,
        "extern \"C\" uint64_t vb_{n}_get(void* p, uint32_t pin) {{"
    )
    .unwrap();
    writeln!(s, "  {class}& t = static_cast<Inst*>(p)->top;").unwrap();
    writeln!(s, "  switch (pin) {{").unwrap();
    for (i, port) in ports.iter().enumerate() {
        writeln!(s, "  case {i}: return t.{};", port.name).unwrap();
    }
    writeln!(s, "  default: return 0;").unwrap();
    writeln!(s, "  }}").unwrap();
    writeln!(s, "}}").unwrap();
    s
}

/// Rust keywords that need `r#` to be used as a method name
const KEYWORDS: &[&str] = &[
    "abstract", "as", "async", "await", "become", "box", "break", "const", "continue", "do", "dyn",
    "else", "enum", "extern", "false", "final", "fn", "for", "gen", "if", "impl", "in", "let",
    "loop", "macro", "match", "mod", "move", "mut", "override", "priv", "pub", "ref", "return",
    "static", "struct", "trait", "true", "try", "type", "typeof", "unsafe", "unsized", "use",
    "virtual", "where", "while", "yield",
];

/// A method name for `name`, as Rust code
fn method_ident(name: &str) -> String {
    if KEYWORDS.contains(&name) {
        format!("r#{name}")
    } else {
        name.to_string()
    }
}

/// Rust side: extern declarations, VTable, pin number constants and the typed `Model`
fn bindings_rs(n: &str, top: &str, ports: &[Port], trace: bool) -> String {
    // every name the module defines must be unique
    let mut items: Vec<String> = ["VTABLE", "Model"].map(String::from).to_vec();
    let mut methods: Vec<String> = [
        "new",
        "eval",
        "raw",
        "raw_mut",
        "default",
        "set_time_ps",
        "eval_before",
        "is_tracing",
        "open_vcd",
        "close_vcd",
    ]
    .map(String::from)
    .to_vec();
    for p in ports {
        assert!(
            !matches!(p.name.as_str(), "self" | "Self" | "super" | "crate" | "_"),
            "model {n}: port `{}` cannot be used as a Rust name",
            p.name
        );
        items.push(p.name.to_ascii_uppercase());
        methods.push(if p.input {
            format!("set_{}", p.name)
        } else {
            p.name.clone()
        });
    }
    for names in [&items, &methods] {
        for (i, a) in names.iter().enumerate() {
            assert!(
                !names[..i].contains(a),
                "model {n}: the generated name `{a}` is used twice; rename a port"
            );
        }
    }

    let mut s = String::new();
    writeln!(s, "/// Generated bindings for `{n}` (top `{top}`)").unwrap();
    writeln!(s, "#[allow(dead_code)]").unwrap();
    writeln!(s, "pub mod {n} {{").unwrap();
    if trace {
        writeln!(
            s,
            "    use ::virtual_bus::verilated::{{RawModel, TraceVTable, VTable}};"
        )
        .unwrap();
        writeln!(s, "    use ::core::ffi::{{c_char, c_void}};").unwrap();
    } else {
        writeln!(s, "    use ::virtual_bus::verilated::{{RawModel, VTable}};").unwrap();
        writeln!(s, "    use ::core::ffi::c_void;").unwrap();
    }
    writeln!(s, "    unsafe extern \"C\" {{").unwrap();
    writeln!(s, "        fn vb_{n}_new() -> *mut c_void;").unwrap();
    writeln!(s, "        fn vb_{n}_free(p: *mut c_void);").unwrap();
    writeln!(s, "        fn vb_{n}_eval(p: *mut c_void);").unwrap();
    writeln!(
        s,
        "        fn vb_{n}_set(p: *mut c_void, pin: u32, v: u64);"
    )
    .unwrap();
    writeln!(s, "        fn vb_{n}_get(p: *mut c_void, pin: u32) -> u64;").unwrap();
    if trace {
        writeln!(
            s,
            "        fn vb_{n}_trace_open(p: *mut c_void, path: *const c_char) -> bool;"
        )
        .unwrap();
        writeln!(s, "        fn vb_{n}_trace_dump(p: *mut c_void, t: u64);").unwrap();
        writeln!(s, "        fn vb_{n}_trace_close(p: *mut c_void);").unwrap();
    }
    writeln!(s, "    }}").unwrap();
    writeln!(s, "    pub static VTABLE: VTable = VTable {{").unwrap();
    writeln!(s, "        name: \"{n}\",").unwrap();
    writeln!(s, "        new: vb_{n}_new,").unwrap();
    writeln!(s, "        free: vb_{n}_free,").unwrap();
    writeln!(s, "        eval: vb_{n}_eval,").unwrap();
    writeln!(s, "        set: vb_{n}_set,").unwrap();
    writeln!(s, "        get: vb_{n}_get,").unwrap();
    if trace {
        writeln!(s, "        trace: Some(TraceVTable {{").unwrap();
        writeln!(s, "            open: vb_{n}_trace_open,").unwrap();
        writeln!(s, "            dump: vb_{n}_trace_dump,").unwrap();
        writeln!(s, "            close: vb_{n}_trace_close,").unwrap();
        writeln!(s, "        }}),").unwrap();
    } else {
        writeln!(s, "        trace: None,").unwrap();
    }
    writeln!(s, "    }};").unwrap();
    for (i, port) in ports.iter().enumerate() {
        let dir = if port.input { "input" } else { "output" };
        writeln!(
            s,
            "    /// {dir} `{}` ({} bit)",
            port.verilog(),
            port.width()
        )
        .unwrap();
        writeln!(
            s,
            "    pub const {}: u32 = {i};",
            port.name.to_ascii_uppercase()
        )
        .unwrap();
    }

    writeln!(
        s,
        "    /// The `{top}` model, with a setter per input and a getter per output"
    )
    .unwrap();
    writeln!(s, "    pub struct Model {{").unwrap();
    writeln!(s, "        raw: RawModel,").unwrap();
    writeln!(s, "    }}").unwrap();
    writeln!(s, "    impl Model {{").unwrap();
    writeln!(
        s,
        "        /// Creates the model. Inputs start at 0 and nothing is evaluated yet"
    )
    .unwrap();
    writeln!(s, "        pub fn new() -> Self {{").unwrap();
    writeln!(s, "            Self {{ raw: RawModel::new(&VTABLE) }}").unwrap();
    writeln!(s, "        }}").unwrap();
    writeln!(
        s,
        "        /// Evaluates the model with the inputs set so far"
    )
    .unwrap();
    writeln!(s, "        pub fn eval(&mut self) {{").unwrap();
    writeln!(s, "            self.raw.eval()").unwrap();
    writeln!(s, "        }}").unwrap();
    writeln!(
        s,
        "        /// The untyped model, to access ports by pin number"
    )
    .unwrap();
    writeln!(s, "        pub fn raw(&self) -> &RawModel {{").unwrap();
    writeln!(s, "            &self.raw").unwrap();
    writeln!(s, "        }}").unwrap();
    writeln!(
        s,
        "        /// The untyped model, to access ports by pin number"
    )
    .unwrap();
    writeln!(s, "        pub fn raw_mut(&mut self) -> &mut RawModel {{").unwrap();
    writeln!(s, "            &mut self.raw").unwrap();
    writeln!(s, "        }}").unwrap();
    // waveform methods. set_time_ps / eval_before / is_tracing exist on every model so that
    // adapters can always forward to them; they do nothing unless a VCD is open
    writeln!(
        s,
        "        /// The bus's simulated time moved (forward `set_time_ps` of the pin model here).\n        /// Only matters while a VCD is open"
    )
    .unwrap();
    writeln!(s, "        pub fn set_time_ps(&mut self, now_ps: u64) {{").unwrap();
    writeln!(s, "            self.raw.set_time_ps(now_ps)").unwrap();
    writeln!(s, "        }}").unwrap();
    writeln!(
        s,
        "        /// Evaluates, and writes the result to the VCD `ps` before the current time\n        /// (for the low half of a system clock)"
    )
    .unwrap();
    writeln!(s, "        pub fn eval_before(&mut self, ps: u64) {{").unwrap();
    writeln!(s, "            self.raw.eval_before(ps)").unwrap();
    writeln!(s, "        }}").unwrap();
    writeln!(s, "        /// Whether a VCD is open").unwrap();
    writeln!(s, "        pub fn is_tracing(&self) -> bool {{").unwrap();
    writeln!(s, "            self.raw.is_tracing()").unwrap();
    writeln!(s, "        }}").unwrap();
    if trace {
        writeln!(
            s,
            "        /// Starts writing a VCD of every signal in `{top}` to `path`"
        )
        .unwrap();
        writeln!(
            s,
            "        pub fn open_vcd(&mut self, path: impl AsRef<::std::path::Path>) -> ::std::io::Result<()> {{"
        )
        .unwrap();
        writeln!(s, "            self.raw.open_vcd(path)").unwrap();
        writeln!(s, "        }}").unwrap();
        writeln!(
            s,
            "        /// Writes the last values and closes the VCD (also done on drop)"
        )
        .unwrap();
        writeln!(s, "        pub fn close_vcd(&mut self) {{").unwrap();
        writeln!(s, "            self.raw.close_vcd()").unwrap();
        writeln!(s, "        }}").unwrap();
    }
    for port in ports {
        let pin = port.name.to_ascii_uppercase();
        let ty = port.rust_type();
        let w = port.width();
        if port.input {
            writeln!(
                s,
                "        /// Sets input `{}` ({w} bit). Takes effect at the next `eval`",
                port.verilog()
            )
            .unwrap();
            writeln!(s, "        pub fn set_{}(&mut self, v: {ty}) {{", port.name).unwrap();
            if ty != "bool" && w < 64 && Some(w) != ty[1..].parse().ok() {
                writeln!(
                    s,
                    "            debug_assert!(u64::from(v) >> {w} == 0, \"`{}` is {w} bits wide: {{v:#x}}\");",
                    port.name
                )
                .unwrap();
            }
            writeln!(s, "            self.raw.set({pin}, u64::from(v))").unwrap();
            writeln!(s, "        }}").unwrap();
        } else {
            writeln!(s, "        /// Output `{}` ({w} bit)", port.verilog()).unwrap();
            writeln!(
                s,
                "        pub fn {}(&self) -> {ty} {{",
                method_ident(&port.name)
            )
            .unwrap();
            if ty == "bool" {
                writeln!(s, "            self.raw.get_bit({pin})").unwrap();
            } else {
                writeln!(s, "            self.raw.get({pin}) as {ty}").unwrap();
            }
            writeln!(s, "        }}").unwrap();
        }
    }
    writeln!(s, "    }}").unwrap();
    writeln!(s, "    impl Default for Model {{").unwrap();
    writeln!(s, "        fn default() -> Self {{").unwrap();
    writeln!(s, "            Self::new()").unwrap();
    writeln!(s, "        }}").unwrap();
    writeln!(s, "    }}").unwrap();
    writeln!(s, "}}").unwrap();
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fresh scratch directory under the system temp dir
    fn scratch(name: &str) -> PathBuf {
        let dir = env::temp_dir().join(format!("vb-build-test-{}-{name}", std::process::id()));
        reset_dir(&dir);
        dir
    }

    fn port(name: &str, msb: u32, lsb: u32, input: bool) -> Port {
        Port {
            name: name.into(),
            msb,
            lsb,
            input,
        }
    }

    /// As Verilator 5.052 writes it (ports grouped by type, not in declaration order)
    const HEADER: &str = "
class alignas(VL_CACHE_LINE_BYTES) Vt VL_NOT_FINAL : public VerilatedModel {
  public:
    VL_IN8(&clk,0,0);
    VL_IN8(&data,7,0);
    VL_IN8(&off,8,1);
    VL_OUT8(&esc__021,0,0);
    VL_OUT16(&type,15,0);
    VL_IN(&word,31,0);
    VL_IN64(&big,40,0);
    VL_UNCOPYABLE(Vt);
";

    #[test]
    fn reads_ports_from_the_header() {
        assert_eq!(
            parse_ports(HEADER).unwrap(),
            [
                port("clk", 0, 0, true),
                port("data", 7, 0, true),
                port("off", 8, 1, true),
                port("esc__021", 0, 0, false),
                port("type", 15, 0, false),
                port("word", 31, 0, true),
                port("big", 40, 0, true),
            ]
        );
        let ports = parse_ports(HEADER).unwrap();
        let types: Vec<_> = ports.iter().map(|p| (p.width(), p.rust_type())).collect();
        assert_eq!(
            types,
            [
                (1, "bool"),
                (8, "u8"),
                (8, "u8"),
                (1, "bool"),
                (16, "u16"),
                (32, "u32"),
                (41, "u64")
            ]
        );
        assert_eq!(ports[2].verilog(), "off[8:1]");
    }

    #[test]
    fn refuses_inout_and_wide_ports() {
        let e = parse_ports("    VL_INOUT8(&sda,0,0);").unwrap_err();
        assert!(e.contains("`sda` is an inout"), "{e}");
        let e = parse_ports("    VL_OUTW(&wide,71,0,3);").unwrap_err();
        assert!(e.contains("`wide` is wider than 64 bits"), "{e}");
    }

    #[test]
    fn bindings_have_typed_methods() {
        let rs = bindings_rs("t", "t", &parse_ports(HEADER).unwrap(), false);
        for expected in [
            "pub const CLK: u32 = 0;",
            "pub const BIG: u32 = 6;",
            "pub fn set_clk(&mut self, v: bool)",
            "pub fn set_off(&mut self, v: u8)",
            "pub fn set_big(&mut self, v: u64)",
            "debug_assert!(u64::from(v) >> 41 == 0",
            "pub fn esc__021(&self) -> bool",
            "pub fn r#type(&self) -> u16",
            "self.raw.get(TYPE) as u16",
        ] {
            assert!(rs.contains(expected), "missing `{expected}` in\n{rs}");
        }
        // full-width ports need no range check
        assert!(!rs.contains(">> 8 == 0"));
        assert!(!rs.contains(">> 32 == 0"));
    }

    #[test]
    #[should_panic(expected = "the generated name `set_clk` is used twice")]
    fn name_clashes_are_reported() {
        bindings_rs(
            "t",
            "t",
            &[port("clk", 0, 0, true), port("set_clk", 0, 0, false)],
            false,
        );
    }

    #[test]
    fn tracing_adds_vcd_functions() {
        let ports = [port("clk", 0, 0, true), port("q", 0, 0, false)];

        let glue = glue_cpp("t", &ports, true);
        for expected in [
            "#include \"verilated_vcd_c.h\"",
            "bool trace_ever_on = (ctx.traceEverOn(true), true);",
            "extern \"C\" bool vb_t_trace_open(void* p, const char* path)",
            "i->vcd->dump(t);",
            "vb_t_trace_close(p); Inst* i",
        ] {
            assert!(glue.contains(expected), "missing `{expected}` in\n{glue}");
        }
        // traceEverOn must come before the model is constructed
        assert!(glue.find("trace_ever_on").unwrap() < glue.find("Vvb_t top;").unwrap());
        assert!(!glue_cpp("t", &ports, false).contains("vcd"));

        let rs = bindings_rs("t", "t", &ports, true);
        assert!(rs.contains("trace: Some(TraceVTable {"));
        assert!(rs.contains("pub fn open_vcd("));
        let plain = bindings_rs("t", "t", &ports, false);
        assert!(plain.contains("trace: None,"));
        assert!(!plain.contains("open_vcd"));
        // adapters can forward the time to any model
        assert!(plain.contains("pub fn set_time_ps("));
        assert!(plain.contains("pub fn eval_before("));
    }

    #[test]
    #[should_panic(expected = "the generated name `eval` is used twice")]
    fn ports_cannot_shadow_model_methods() {
        bindings_rs("t", "t", &[port("eval", 0, 0, false)], false);
    }

    #[test]
    fn reads_inputs_from_verilator_depfile() {
        let d =
            "/o/Vvb_x.cpp /o/Vvb_x.h  : /bin/verilator_bin /bin/verilator_bin /rtl/b.v /rtl/a.v \n";
        assert_eq!(
            verilator_inputs(d),
            ["/bin/verilator_bin", "/rtl/a.v", "/rtl/b.v"].map(PathBuf::from)
        );
        assert!(verilator_inputs("garbage").is_empty());
    }

    #[test]
    fn stamp_notices_changed_settings_and_inputs() {
        let dir = scratch("stamp");
        let rtl = dir.join("a.v");
        fs::write(&rtl, "module a; endmodule\n").unwrap();
        Stamp::new(7, vec![rtl.clone()]).write(&dir);

        let stamp = Stamp::read(&dir).unwrap();
        assert!(stamp.is_current(7));
        assert!(!stamp.is_current(8), "settings changed");

        // touching without changing the content keeps the cache
        fs::write(&rtl, "module a; endmodule\n").unwrap();
        assert!(Stamp::read(&dir).unwrap().is_current(7));

        fs::write(&rtl, "module a; wire w; endmodule\n").unwrap();
        assert!(!Stamp::read(&dir).unwrap().is_current(7), "content changed");

        fs::remove_file(&rtl).unwrap();
        assert!(!Stamp::read(&dir).unwrap().is_current(7), "input deleted");
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn verilator_binary_is_not_hashed() {
        assert_eq!(hash_input(Path::new("/nonexistent/verilator_bin")), None);
        assert_eq!(fnv1a(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(fnv1a(b"a"), 0xaf63_dc4c_8601_ec8c);
    }
}
