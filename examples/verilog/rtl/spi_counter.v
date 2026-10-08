// spi_counter: an SPI timer/counter
//
//   SPI mode 0. First byte: bit7 = R(1)/W(0), bit6..0 = address; auto-increment after that
//   0x00 ID       = 0x5C (RO)
//   0x01 CTRL     bit0 EN, bit1 CLR (write 1 to clear COUNT, reads 0), bit2 IRQ_EN
//   0x02 PRESCALE COUNT increments every PRESCALE + 1 clk cycles
//   0x03 COUNT    (RO)
//   0x04 COMPARE  MATCH is set when COUNT reaches this value (reset value 0xFF)
//   0x05 STATUS   bit0 MATCH (write 1 to clear)
//   Other addresses read 0x00; writes to them are ignored
//
// SCK / CS_N / MOSI are synchronized to clk (50 MHz assumed) with two flip-flops.
// MISO is split into miso and miso_oe (no inout). irq = MATCH && IRQ_EN.
//
// Because the inputs are sampled with clk, the master must hold each SCK level and keep CS high
// between frames for a few clk periods (SCK up to about clk / 8, CS high at least 3 clk).
// A shorter CS high is not seen, and two frames run together as one.
`default_nettype none

module spi_counter #(
    parameter [7:0] ID = 8'h5C
) (
    input  wire clk,
    input  wire rst_n,
    input  wire cs_n,
    input  wire sck,
    input  wire mosi,
    output wire miso,
    output wire miso_oe,
    output wire irq
);

  localparam [6:0] REG_ID       = 7'h00;
  localparam [6:0] REG_CTRL     = 7'h01;
  localparam [6:0] REG_PRESCALE = 7'h02;
  localparam [6:0] REG_COUNT    = 7'h03;
  localparam [6:0] REG_COMPARE  = 7'h04;
  localparam [6:0] REG_STATUS   = 7'h05;

  // two-stage synchronizer + one stage of the previous value
  reg [2:0] sck_s;
  reg [1:0] cs_s;
  reg [1:0] mosi_s;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sck_s  <= 3'b000;
      cs_s   <= 2'b11;
      mosi_s <= 2'b00;
    end else begin
      sck_s  <= {sck_s[1:0], sck};
      cs_s   <= {cs_s[0], cs_n};
      mosi_s <= {mosi_s[0], mosi};
    end
  end

  wire sck_rise = sck_s[1] & ~sck_s[2];
  wire sck_fall = ~sck_s[1] & sck_s[2];
  wire selected = ~cs_s[1];
  wire mosi_q   = mosi_s[1];

  // registers and counter
  reg [7:0] ctrl;
  reg [7:0] prescale;
  reg [7:0] compare;
  reg [7:0] count;
  reg [7:0] div;
  reg       match;

  // SPI frame state
  reg [2:0] bitcnt;
  reg [6:0] shreg;     // the upper 7 bits received so far
  reg       cmd_done;  // the first byte has been received
  reg       rd;
  reg [6:0] addr;
  reg [7:0] tx;

  wire [7:0] rx_byte = {shreg, mosi_q};
  wire [7:0] count_next = count + 8'd1;

  reg [7:0] rdata;
  always @(*) begin
    case (addr)
      REG_ID:       rdata = ID;
      REG_CTRL:     rdata = ctrl;
      REG_PRESCALE: rdata = prescale;
      REG_COUNT:    rdata = count;
      REG_COMPARE:  rdata = compare;
      REG_STATUS:   rdata = {7'b0, match};
      default:      rdata = 8'h00;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ctrl     <= 8'h00;
      prescale <= 8'h00;
      compare  <= 8'hFF;
      count    <= 8'h00;
      div      <= 8'h00;
      match    <= 1'b0;
      bitcnt   <= 3'd0;
      shreg    <= 7'h00;
      cmd_done <= 1'b0;
      rd       <= 1'b0;
      addr     <= 7'h00;
      tx       <= 8'h00;
    end else begin
      // ---- counter ----
      if (ctrl[0]) begin
        if (div == prescale) begin
          div   <= 8'h00;
          count <= count_next;
          if (count_next == compare) match <= 1'b1;
        end else begin
          div <= div + 8'd1;
        end
      end

      // ---- SPI (assignments here win over the counter in the same cycle) ----
      if (!selected) begin
        bitcnt   <= 3'd0;
        cmd_done <= 1'b0;
        tx       <= 8'h00;
      end else if (sck_rise) begin
        // capture MOSI
        shreg  <= rx_byte[6:0];
        bitcnt <= bitcnt + 3'd1;
        if (bitcnt == 3'd7) begin
          if (!cmd_done) begin
            cmd_done <= 1'b1;
            rd       <= rx_byte[7];
            addr     <= rx_byte[6:0];
          end else begin
            if (!rd) begin
              case (addr)
                REG_CTRL: begin
                  ctrl <= rx_byte & 8'hFD;  // CLR is not stored
                  if (rx_byte[1]) begin
                    count <= 8'h00;
                    div   <= 8'h00;
                  end
                end
                REG_PRESCALE: prescale <= rx_byte;
                REG_COMPARE:  compare  <= rx_byte;
                REG_STATUS:   if (rx_byte[0]) match <= 1'b0;
                default: ;
              endcase
            end
            addr <= addr + 7'd1;
          end
        end
      end else if (sck_fall) begin
        // update MISO. At a byte boundary (bitcnt == 0) load the next byte
        if (bitcnt == 3'd0) begin
          tx <= (cmd_done && rd) ? rdata : 8'h00;
        end else begin
          tx <= {tx[6:0], 1'b0};
        end
      end
    end
  end

  assign miso    = tx[7];
  assign miso_oe = selected;
  assign irq     = match & ctrl[2];

endmodule

`default_nettype wire
