// Simulation-only wrapper: instantiates spi_whoami_top (no output enable) and
// resolves the MISO line (pulled up when undriven) inside. Not for synthesis.
//
//   vdd        : power. gnd is tied to 0 inside this wrapper
//   miso_level : the resolved MISO level. Whether the DUT drives 1 or is Hi-Z cannot be told apart
`default_nettype none

module spi_whoami_sim (
    input  wire vdd,
    input  wire cs_n,
    input  wire sck,
    input  wire mosi,
    output wire miso_level
);

  tri1 miso;  // pull-up resistor

  spi_whoami_top dut (
      .vdd(vdd),
      .gnd(1'b0),
      .cs_n(cs_n),
      .sck(sck),
      .mosi(mosi),
      .miso(miso)
  );

  assign miso_level = miso;

endmodule

`default_nettype wire
