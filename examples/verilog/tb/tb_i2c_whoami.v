// Unit testbench for i2c_whoami (Icarus Verilog)
`timescale 1ns / 1ps
`default_nettype none

module tb_i2c_whoami;

  localparam integer QUARTER = 625;  // 400 kHz

  reg rst_n = 1'b0;
  reg m_scl_low = 1'b0;
  reg m_sda_low = 1'b0;
  wire scl = ~m_scl_low;

  // open drain: SDA is low if the master or the slave pulls it
  wire s_sda_low;
  wire sda = ~(m_sda_low | s_sda_low);
  i2c_whoami dut (
      .rst_n(rst_n),
      .scl(scl),
      .sda_i(sda),
      .sda_low(s_sda_low)
  );

  integer errors = 0;

  task i2c_start;
    begin
      m_sda_low = 1'b0; m_scl_low = 1'b0; #(2*QUARTER);
      m_sda_low = 1'b1; #(2*QUARTER);
      m_scl_low = 1'b1; #(QUARTER);
    end
  endtask

  task i2c_stop;
    begin
      m_sda_low = 1'b1; #(QUARTER);
      m_scl_low = 1'b0; #(2*QUARTER);
      m_sda_low = 1'b0; #(2*QUARTER);
    end
  endtask

  task i2c_bit_out(input b);
    begin
      m_sda_low = ~b; #(QUARTER);
      m_scl_low = 1'b0; #(2*QUARTER);
      m_scl_low = 1'b1; #(QUARTER);
    end
  endtask

  task i2c_bit_in(output b);
    begin
      m_sda_low = 1'b0; #(QUARTER);
      m_scl_low = 1'b0; #(QUARTER);
      b = sda; #(QUARTER);
      m_scl_low = 1'b1; #(QUARTER);
    end
  endtask

  task i2c_write_byte(input [7:0] d, output ack);
    integer i;
    reg b;
    begin
      for (i = 7; i >= 0; i = i - 1) i2c_bit_out(d[i]);
      i2c_bit_in(b);
      ack = ~b;
    end
  endtask

  task i2c_read_byte(input ack, output [7:0] d);
    integer i;
    reg b;
    begin
      for (i = 7; i >= 0; i = i - 1) begin
        i2c_bit_in(b);
        d[i] = b;
      end
      i2c_bit_out(~ack);
    end
  endtask

  task expect_ack(input ack, input [255:0] what);
    if (!ack) begin
      $display("FAIL: NACK at %0s", what);
      errors = errors + 1;
    end
  endtask

  // read one byte from ptr
  task read_reg(input [7:0] ptr, output [7:0] d, output ok);
    reg a0, a1, a2;
    begin
      i2c_start;
      i2c_write_byte({7'h29, 1'b0}, a0);
      i2c_write_byte(ptr, a1);
      i2c_start;
      i2c_write_byte({7'h29, 1'b1}, a2);
      i2c_read_byte(1'b0, d);
      i2c_stop;
      ok = a0 & a1 & a2;
    end
  endtask

  reg ack;
  reg [7:0] d0, d1;

  initial begin
    // make sure the asynchronous reset gets a falling edge (the initial value at time 0 alone may not count
    // as an edge, and then a circuit without a clock is never reset)
    #10 rst_n = 1'b1;
    #10 rst_n = 1'b0;
    #100 rst_n = 1'b1;
    #200;

    // sequential read of WHO_AM_I and SCRATCH
    i2c_start;
    i2c_write_byte({7'h29, 1'b0}, ack); expect_ack(ack, "addr W");
    i2c_write_byte(8'h0F, ack);          expect_ack(ack, "ptr");
    i2c_start;
    i2c_write_byte({7'h29, 1'b1}, ack); expect_ack(ack, "addr R");
    i2c_read_byte(1'b1, d0);
    i2c_read_byte(1'b0, d1);
    i2c_stop;
    if (d0 !== 8'hA5 || d1 !== 8'h00) begin
      $display("FAIL: read %02x %02x, expected a5 00", d0, d1);
      errors = errors + 1;
    end

    // write SCRATCH and read it back
    i2c_start;
    i2c_write_byte({7'h29, 1'b0}, ack); expect_ack(ack, "addr W");
    i2c_write_byte(8'h10, ack);          expect_ack(ack, "ptr");
    i2c_write_byte(8'h5C, ack);          expect_ack(ack, "data");
    i2c_stop;
    i2c_start;
    i2c_write_byte({7'h29, 1'b0}, ack); expect_ack(ack, "addr W");
    i2c_write_byte(8'h10, ack);          expect_ack(ack, "ptr");
    i2c_start;
    i2c_write_byte({7'h29, 1'b1}, ack); expect_ack(ack, "addr R");
    i2c_read_byte(1'b0, d0);
    i2c_stop;
    if (d0 !== 8'h5C) begin
      $display("FAIL: scratch %02x, expected 5c", d0);
      errors = errors + 1;
    end

    // another address is NACKed
    i2c_start;
    i2c_write_byte({7'h30, 1'b0}, ack);
    i2c_stop;
    if (ack) begin
      $display("FAIL: wrong address was ACKed");
      errors = errors + 1;
    end

    // no response in reset
    rst_n = 1'b0;
    #200;
    i2c_start;
    i2c_write_byte({7'h29, 1'b0}, ack);
    i2c_stop;
    if (ack) begin
      $display("FAIL: ACKed while in reset");
      errors = errors + 1;
    end

    // after reset, SCRATCH is back to 0
    rst_n = 1'b1;
    #1000;
    read_reg(8'h10, d0, ack);
    if (!ack || d0 !== 8'h00) begin
      $display("FAIL: scratch after reset = %02x (ack %0d), expected 00", d0, ack);
      errors = errors + 1;
    end

    if (errors == 0) $display("PASS tb_i2c_whoami");
    $finish;
  end

endmodule

`default_nettype wire
