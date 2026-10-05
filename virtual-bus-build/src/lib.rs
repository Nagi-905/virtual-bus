//! A helper called from `build.rs`. Converts Verilog to C++ with Verilator, generates a C ABI glue layer
//! and the Rust bindings (pin number constants and a `VTable`), and links them.
//!
//! No hand-written C++ needed. To add a port, just add an `input` / `output` to the [`Model`].
//!
//! ```ignore
//! // build.rs
//! fn main() {
//!     virtual_bus_build::Verilated::new()
//!         .rtl_dir("rtl")
//!         .model(
//!             virtual_bus_build::Model::new("spi_whoami", "spi_whoami")
//!                 .source("spi_whoami.v")
//!                 .flag("-Wall")
//!                 .input("rst_n", 1)
//!                 .input("cs_n", 1)
//!                 .input("sck", 1)
//!                 .input("mosi", 1)
//!                 .output("miso", 1)
//!                 .output("miso_oe", 1),
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
//! // pass bindings::spi_whoami::VTABLE to virtual_bus::verilated::RawModel::new,
//! // then read and write with pin numbers such as bindings::spi_whoami::CS_N
//! ```
//!
//! Requirements: Verilator 5.x (`verilator` on PATH, or `VERILATOR_ROOT`) and a C++17 compiler.
//! The using crate must depend on `virtual-bus` (the bindings refer to
//! `virtual_bus::verilated::VTable`).
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
    ports: Vec<Port>,
}

#[derive(Debug, Clone)]
struct Port {
    name: String,
    width: u32,
    input: bool,
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
            ports: Vec::new(),
        }
    }

    /// A Verilog source (relative to [`Verilated::rtl_dir`], or an absolute path)
    pub fn source(mut self, path: impl AsRef<Path>) -> Self {
        self.sources.push(path.as_ref().to_path_buf());
        self
    }

    /// Extra flags passed to verilator (`-Wall`, `-Wno-fatal` and so on)
    pub fn flag(mut self, flag: &str) -> Self {
        self.flags.push(flag.to_string());
        self
    }

    /// An input port, 1..=64 bits wide
    pub fn input(self, name: &str, width: u32) -> Self {
        self.port(name, width, true)
    }

    /// An output port, 1..=64 bits wide
    pub fn output(self, name: &str, width: u32) -> Self {
        self.port(name, width, false)
    }

    fn port(mut self, name: &str, width: u32, input: bool) -> Self {
        assert!(
            (1..=64).contains(&width),
            "port width must be 1..=64: {name}"
        );
        self.ports.push(Port {
            name: name.to_string(),
            width,
            input,
        });
        self
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
        for lib in models.iter().chain([&runtime]) {
            println!("cargo::rustc-link-search=native={}", lib.dir.display());
            println!("cargo::rustc-link-lib=static={}", lib.name);
        }
        if let Some(stdlib) = tc.cxx_stdlib() {
            println!("cargo::rustc-link-lib={stdlib}");
        }

        let mut rs = String::from("// Generated by virtual-bus-build. Do not edit\n");
        for m in &self.models {
            rs.push_str(&bindings_rs(m));
        }
        fs::write(out.join("verilated_models.rs"), rs).unwrap();
    }
}

/// Bump when the way the C++ is compiled changes, so that cached builds are thrown away
/// (changes to the glue are noticed without this)
const CACHE_REV: u32 = 1;

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
    fn cc(&self, dir: &Path) -> cc::Build {
        let mut cc = cc::Build::new();
        cc.cpp(true)
            .std("c++17")
            .opt_level(2) // optimize the simulation even in debug builds
            .warnings(false)
            .include(&self.include)
            .include(self.include.join("vltstd"))
            .define("VM_COVERAGE", "0")
            .define("VM_SC", "0")
            .define("VM_TRACE", "0")
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

/// The Verilator runtime (`verilated.cpp` and friends), compiled once per toolchain
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
    tc.cc(&dir)
        .file(tc.include.join("verilated.cpp"))
        .file(tc.include.join("verilated_threads.cpp"))
        .compile(&lib.name);
    Stamp::new(settings, Vec::new()).write(&dir);
    lib
}

/// One model: verilator, the glue and its C++, unless nothing it depends on changed
fn build_model(m: &Model, rtl: &Path, tc: &Toolchain, out: &Path, pkg: &str) -> Lib {
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
    let glue = glue_cpp(m);
    let settings = hash_str(&format!("{}{m:?}\n{sources:?}\n{glue}", tc.key));

    if lib.exists() {
        if let Some(stamp) = Stamp::read(&dir).filter(|s| s.is_current(settings)) {
            stamp.watch();
            return lib;
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
        .args(&m.flags)
        .args(&sources);
    let st = cmd.status().expect("failed to start verilator");
    assert!(st.success(), "verilator failed: {}", m.name);

    fs::write(dir.join(format!("vb_{}_glue.cpp", m.name)), glue).unwrap();
    let mut cc = tc.cc(&dir);
    cc.include(&dir).files(cpp_files(&dir)).compile(&lib.name);

    let depfile = fs::read_to_string(dir.join(format!("Vvb_{}__ver.d", m.name)))
        .expect("verilator wrote no dependency file");
    let stamp = Stamp::new(settings, verilator_inputs(&depfile));
    stamp.write(&dir);
    stamp.watch();
    lib
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

/// The C ABI glue (new / free / eval / set / get)
fn glue_cpp(m: &Model) -> String {
    let class = format!("Vvb_{}", m.name);
    let n = &m.name;
    let mut s = String::new();
    writeln!(s, "// Generated by virtual-bus-build. Do not edit").unwrap();
    writeln!(s, "#include <cstdint>").unwrap();
    writeln!(s, "#include \"verilated.h\"").unwrap();
    writeln!(s, "#include \"{class}.h\"").unwrap();
    writeln!(s, "namespace {{").unwrap();
    writeln!(s, "struct Inst {{").unwrap();
    writeln!(s, "  VerilatedContext ctx;").unwrap();
    writeln!(s, "  {class} top;").unwrap();
    writeln!(s, "  Inst() : ctx(), top(&ctx, \"top\") {{}}").unwrap();
    writeln!(s, "}};").unwrap();
    writeln!(s, "}}").unwrap();
    writeln!(
        s,
        "extern \"C\" void* vb_{n}_new() {{ return new Inst(); }}"
    )
    .unwrap();
    writeln!(
        s,
        "extern \"C\" void vb_{n}_free(void* p) {{ Inst* i = static_cast<Inst*>(p); i->top.final(); delete i; }}"
    )
    .unwrap();
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
    for (i, port) in m.ports.iter().enumerate() {
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
    for (i, port) in m.ports.iter().enumerate() {
        writeln!(s, "  case {i}: return t.{};", port.name).unwrap();
    }
    writeln!(s, "  default: return 0;").unwrap();
    writeln!(s, "  }}").unwrap();
    writeln!(s, "}}").unwrap();
    s
}

/// Rust side: extern declarations, VTable, pin number constants
fn bindings_rs(m: &Model) -> String {
    let n = &m.name;
    let mut s = String::new();
    writeln!(s, "/// Generated bindings for `{}` (top `{}`)", n, m.top).unwrap();
    writeln!(s, "pub mod {n} {{").unwrap();
    writeln!(s, "    use ::virtual_bus::verilated::VTable;").unwrap();
    writeln!(s, "    use ::core::ffi::c_void;").unwrap();
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
    writeln!(s, "    }}").unwrap();
    writeln!(s, "    pub static VTABLE: VTable = VTable {{").unwrap();
    writeln!(s, "        name: \"{n}\",").unwrap();
    writeln!(s, "        new: vb_{n}_new,").unwrap();
    writeln!(s, "        free: vb_{n}_free,").unwrap();
    writeln!(s, "        eval: vb_{n}_eval,").unwrap();
    writeln!(s, "        set: vb_{n}_set,").unwrap();
    writeln!(s, "        get: vb_{n}_get,").unwrap();
    writeln!(s, "    }};").unwrap();
    for (i, port) in m.ports.iter().enumerate() {
        let dir = if port.input { "input" } else { "output" };
        writeln!(s, "    /// {dir} `{}` ({} bit)", port.name, port.width).unwrap();
        writeln!(
            s,
            "    pub const {}: u32 = {i};",
            port.name.to_ascii_uppercase()
        )
        .unwrap();
    }
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
