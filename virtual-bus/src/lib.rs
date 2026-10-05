//! Virtual buses for running embedded-hal 1.0 I2C / SPI drivers against IC models on your PC.
//!
//! Drivers only use the embedded-hal traits, so passing the types from this crate instead of a real bus is all it takes.
//!
//! ```
//! use embedded_hal::i2c::I2c;
//! use virtual_bus::VirtualI2cBus;
//! use virtual_bus::devices::register::{I2cFormat, RegisterMap};
//!
//! let regs = RegisterMap::new().ro(0x0F, 0xA5).rw(0x10, 0x00);
//! let mut bus = VirtualI2cBus::new();
//! bus.attach(0x29, regs.i2c(I2cFormat::new())).unwrap();
//!
//! let mut who = [0u8];
//! bus.write_read(0x29, &[0x0F], &mut who).unwrap();
//! assert_eq!(who[0], 0xA5);
//! ```
//!
//! # Where to start
//!
//! | To do this | Use |
//! |---|---|
//! | Build a model from a list of registers | [`devices::register::RegisterMap`] |
//! | Write the behavior yourself | Implement [`I2cSlave`] or [`bus::spi::SpiSlave`] |
//! | Run fast, byte by byte | [`VirtualI2cBus`], [`bus::spi::VirtualSpiDevice`], [`bus::spi::VirtualSpiBus`] |
//! | Simulate signal lines and timing | [`bus::i2c::sim::SimI2cBus`], [`bus::spi::sim::SimSpiBus`] |
//! | Put a Rust model on the signal lines | [`bus::i2c::sim::PinLevelI2cSlave`], [`bus::spi::sim::PinLevelSpiSlave`] |
//! | Attach Verilog RTL | See "Attaching Verilog" below |
//! | Inject faults such as NACKs | [`VirtualI2cBus::inject_fault`], [`bus::spi::VirtualSpiDevice::inject_fault`] |
//!
//! Runnable examples are in `virtual-bus/examples/` in the repository.
//!
//! # Attaching Verilog
//!
//! 1. Call `virtual-bus-build` from `build.rs` with your RTL files and port list.
//!    It runs Verilator and generates the code needed to call the model from Rust
//! 2. Include the generated code and write a small adapter that reads and writes pins through
//!    [`verilated::RawModel`]
//! 3. Implement [`bus::i2c::sim::I2cPinModel`] / [`bus::spi::sim::SpiPinModel`] for the adapter and
//!    attach it to the signal lines
//!
//! See `examples/rtl-demo` in the repository for a working example.
//! With [`VirtualI2cBus::attach_i2c`], Rust models and RTL can share one bus.

pub mod bus;
pub mod devices;
pub mod verilated;

pub use bus::i2c::{AttachError, Direction, Fault, I2cSlave, LogEntry, LogOp, Nack, VirtualI2cBus};
pub use bus::{Shared, SimDelay, shared};
