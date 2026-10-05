//! Building blocks for models to attach to a bus.
//!
//! | Building block | Bus | What it is |
//! |---|---|---|
//! | [`register::RegisterMap`] | SPI / I2C | A register map for devices where the first byte selects the address and the rest reads or writes. SPI and I2C front ends can be attached |
//!
//! Models of specific ICs are not part of this crate. See `examples/mcp23017` for how to write one.
//! RTL models are built in the user's crate with `virtual-bus-build`
//! (see `examples/rtl-demo` in the repository).

pub mod register;
