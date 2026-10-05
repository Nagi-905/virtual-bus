// spi_whoami: a minimal SPI slave
//
//   First byte: bit7 = R(1)/W(0), bit6 = MS (auto-increment), bit5:0 = address
//   0x0F WHO_AM_I = 0x33 (RO)
//   0x20 CTRL           (RW)
//   Other addresses read 0x00; writes to them are ignored
//
// SPI modes 0 / 3 only. Clocked directly by SCK (no system clock).
//   SCK rising edge: capture MOSI
//   SCK falling edge: update MISO
// CS_N = 1 asynchronously resets the frame state. Does not work with mode 1 / 2 masters.
// RST_N = 0 (power-on reset) resets everything, CTRL included.
//
// MISO is split into the output miso and the output enable miso_oe; the bus is resolved
// by the simulator (Rust side).
`default_nettype none

module spi_whoami (
    input  wire rst_n,
    input  wire cs_n,
    input  wire sck,
    input  wire mosi,
    output wire miso,
    output wire miso_oe
);

  localparam [5:0] REG_WHO_AM_I = 6'h0F;
  localparam [5:0] REG_CTRL     = 6'h20;
  localparam [7:0] WHO_AM_I     = 8'h33;

  reg [2:0] bitcnt;
  reg [6:0] shreg;     // upper 7 bits received so far
  reg       cmd_done;  // the first byte has been received
  reg       rd;
  reg       ms;
  reg [5:0] addr;
  reg [7:0] ctrl;
  reg [7:0] tx;

  wire [7:0] rx_byte = {shreg, mosi};

  reg [7:0] rdata;
  always @(*) begin
    case (addr)
      REG_WHO_AM_I: rdata = WHO_AM_I;
      REG_CTRL:     rdata = ctrl;
      default:      rdata = 8'h00;
    endcase
  end

  // the frame state is reset by both CS_N = 1 and RST_N = 0
  wire frame_rst = cs_n | ~rst_n;

  // CTRL is kept across CS, so only RST_N resets it.
  // bitcnt stays 0 while CS_N = 1, so there is no need to look at cs_n
  always @(posedge sck or negedge rst_n) begin
    if (!rst_n) begin
      ctrl <= 8'h00;
    end else if (bitcnt == 3'd7 && cmd_done && !rd && addr == REG_CTRL) begin
      ctrl <= rx_byte;
    end
  end

  // receive side: SCK rising edge
  always @(posedge sck or posedge frame_rst) begin
    if (frame_rst) begin
      bitcnt   <= 3'd0;
      shreg    <= 7'h00;
      cmd_done <= 1'b0;
      rd       <= 1'b0;
      ms       <= 1'b0;
      addr     <= 6'h00;
    end else begin
      shreg  <= rx_byte[6:0];
      bitcnt <= bitcnt + 3'd1;
      if (bitcnt == 3'd7) begin
        if (!cmd_done) begin
          cmd_done <= 1'b1;
          rd       <= rx_byte[7];
          ms       <= rx_byte[6];
          addr     <= rx_byte[5:0];
        end else if (ms) begin
          addr <= addr + 6'd1;
        end
      end
    end
  end

  // transmit side: SCK falling edge. Loads the next byte at byte boundaries (bitcnt == 0)
  always @(negedge sck or posedge frame_rst) begin
    if (frame_rst) begin
      tx <= 8'h00;
    end else if (bitcnt == 3'd0) begin
      tx <= (cmd_done && rd) ? rdata : 8'h00;
    end else begin
      tx <= {tx[6:0], 1'b0};
    end
  end

  assign miso    = tx[7];
  assign miso_oe = ~frame_rst;

endmodule

`default_nettype wire
