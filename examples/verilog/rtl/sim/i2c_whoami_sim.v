// Simulation-only wrapper: instantiates i2c_whoami_top (no output enable) and
// resolves the SDA line (pull-up and wired-AND) inside. Not for synthesis.
//
//   vdd         : power. gnd is tied to 0 inside this wrapper
//   ext_sda_low : someone other than the DUT (the master or another device) pulls SDA low
//   sda_level   : the resolved SDA level
//
// Whether the DUT itself pulls SDA is not visible from outside (there is no output enable).
`default_nettype none

module i2c_whoami_sim (
    input  wire vdd,
    input  wire scl,
    input  wire ext_sda_low,
    output wire sda_level
);

  tri1 sda;  // pull-up resistor

  assign sda = ext_sda_low ? 1'b0 : 1'bz;

  i2c_whoami_top dut (
      .vdd(vdd),
      .gnd(1'b0),
      .scl(scl),
      .sda(sda)
  );

  assign sda_level = sda;

endmodule

`default_nettype wire
