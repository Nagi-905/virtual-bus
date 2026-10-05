// Unit testbench for spi_whoami (Icarus Verilog). Tries mode 0 and mode 3
`timescale 1ns / 1ps
`default_nettype none

module tb_spi_whoami;

  localparam integer HALF = 250;

  // the reset pin for the core, vdd (power) for the top. Goes x → 0 at time 1 to apply reset.
  // CS_N also goes 0 → 1 at time 1, giving the frame state a rising edge of its asynchronous reset
  reg rst_n;
  reg cs_n = 1'b0;
  reg sck = 1'b0;
  reg mosi = 1'b0;
  reg cpol = 1'b0;
`ifdef USE_TOP
  // chip top without output enable. MISO is resolved on a line with a pull-up
  tri1 miso_line;
  spi_whoami_top dut (
      .vdd(rst_n),
      .gnd(1'b0),
      .cs_n(cs_n),
      .sck(sck),
      .mosi(mosi),
      .miso(miso_line)
  );
`else
  wire miso;
  wire miso_oe;
  wire miso_line = miso_oe ? miso : 1'b1;  // pulled up when undriven
  spi_whoami dut (
      .rst_n(rst_n),
      .cs_n(cs_n),
      .sck(sck),
      .mosi(mosi),
      .miso(miso),
      .miso_oe(miso_oe)
  );
`endif

  integer errors = 0;

  // one-byte transfer with CPHA = 0 (both mode 0 / 3 capture on the rising edge)
  task xfer(input [7:0] out, output [7:0] in);
    integer i;
    begin
      for (i = 7; i >= 0; i = i - 1) begin
        if (cpol) begin
          sck = 1'b0;  // mode 3: change MOSI on the falling edge
          mosi = out[i];
          #(HALF);
          in[i] = miso_line;
          sck = 1'b1;
          #(HALF);
        end else begin
          mosi = out[i];
          #(HALF);
          in[i] = miso_line;
          sck = 1'b1;
          #(HALF);
          sck = 1'b0;
        end
      end
    end
  endtask

  task check(input [7:0] got, input [7:0] exp, input [255:0] what);
    if (got !== exp) begin
      $display("FAIL (cpol=%0d) %0s: got %02x, expected %02x", cpol, what, got, exp);
      errors = errors + 1;
    end
  endtask

  reg [7:0] r0, r1, dummy;
  integer m;

  initial begin
    #1 rst_n = 1'b0;
    cs_n = 1'b1;
    #1 rst_n = 1'b1;
    for (m = 0; m < 2; m = m + 1) begin
      cpol = m[0];
      sck = cpol;
      #(HALF);

      // WHO_AM_I
      cs_n = 1'b0; #(HALF);
      xfer(8'h8F, dummy);
      xfer(8'h00, r0);
      cs_n = 1'b1; #(HALF);
      check(r0, 8'h33, "WHO_AM_I");

      // write CTRL
      cs_n = 1'b0; #(HALF);
      xfer(8'h20, dummy);
      xfer(8'h47 + m[7:0], dummy);
      cs_n = 1'b1; #(HALF);

      // read 2 bytes from 0x1F with the MS bit
      cs_n = 1'b0; #(HALF);
      xfer(8'hDF, dummy);
      xfer(8'h00, r0);
      xfer(8'h00, r1);
      cs_n = 1'b1; #(HALF);
      check(r0, 8'h00, "0x1F");
      check(r1, 8'h47 + m[7:0], "CTRL");
    end

    // in reset (power off for the top), MISO is not driven even with CS low
    rst_n = 1'b0;
    cs_n = 1'b0; #(HALF);
    xfer(8'h8F, dummy);
    xfer(8'h00, r0);
    cs_n = 1'b1; #(HALF);
    check(r0, 8'hFF, "unpowered MISO (pull-up)");

    // after a power cycle (reset release for the core), CTRL is back to 0
    rst_n = 1'b1; #(HALF);
    cs_n = 1'b0; #(HALF);
    xfer(8'hA0, dummy);
    xfer(8'h00, r0);
    cs_n = 1'b1; #(HALF);
    check(r0, 8'h00, "CTRL after reset");

`ifdef USE_TOP
    if (errors == 0) $display("PASS tb_spi_whoami (top without output enable)");
`else
    if (errors == 0) $display("PASS tb_spi_whoami (core)");
`endif
    $finish;
  end

endmodule

`default_nettype wire
