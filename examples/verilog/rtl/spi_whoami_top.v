// Chip top of spi_whoami: no output enable on the outside; MISO is a tri-state output.
// No reset pin: a power-on reset is created from vdd / gnd.
//
// The core (spi_whoami) keeps separate miso / miso_oe. The tri-state is applied only at this level.
// This chip has no system clock, so the POR is simply "released while powered", with
// no delay (all registers, CTRL included, take their reset values the moment power comes on).
// MISO is Hi-Z while unpowered.
// In simulation, wrap it with sim/spi_whoami_sim.v (the top has an inout, like i2c_whoami_top.v).
`default_nettype none

module spi_whoami_top (
    input  wire vdd,
    input  wire gnd,
    input  wire cs_n,
    input  wire sck,
    input  wire mosi,
    output wire miso
);

  wire por_n = vdd & ~gnd;
  wire miso_o;
  wire miso_oe;

  spi_whoami core (
      .rst_n(por_n),
      .cs_n(cs_n),
      .sck(sck),
      .mosi(mosi),
      .miso(miso_o),
      .miso_oe(miso_oe)
  );

  assign miso = miso_oe ? miso_o : 1'bz;

endmodule

`default_nettype wire
