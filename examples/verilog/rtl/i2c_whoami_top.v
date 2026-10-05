// Chip top of i2c_whoami: no output enable on the outside; SDA is an open-drain inout.
// No reset pin: a power-on reset is created from vdd / gnd (same shape as spi_whoami_top).
// Reset is released only while power is on, with no delay. While unpowered, the core stays
// in reset and does not pull SDA (it does not ACK its address).
//
// The core (i2c_whoami) keeps separate sda_i / sda_low. The tri-state is applied only at this level
// (where the pad cells would be on a real chip). In simulation, wrap it with sim/i2c_whoami_sim.v.
`default_nettype none

module i2c_whoami_top #(
    parameter [6:0] ADDR = 7'h29
) (
    input  wire vdd,
    input  wire gnd,
    input  wire clk,
    input  wire scl,
    inout  wire sda
);

  wire por_n = vdd & ~gnd;
  wire sda_low;

  i2c_whoami #(
      .ADDR(ADDR)
  ) core (
      .clk(clk),
      .rst_n(por_n),
      .scl(scl),
      .sda_i(sda),
      .sda_low(sda_low)
  );

  assign sda = sda_low ? 1'b0 : 1'bz;

endmodule

`default_nettype wire
