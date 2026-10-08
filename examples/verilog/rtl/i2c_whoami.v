// i2c_whoami: a minimal I2C slave without a system clock, running on SCL and SDA only
//
//   7-bit address ADDR (default 0x29)
//   0x0F WHO_AM_I  = 0xA5 (RO)
//   0x10 SCRATCH         (RW)
//   Other addresses read 0x00; writes to them are ignored
//
// The first byte of a write transaction is the register pointer. The pointer
// auto-increments with each data byte.
//
// - Finds START using the falling edge of SDA as a clock (only while SCL is high)
// - Captures SDA on the rising edge of SCL and changes the SDA output on the falling edge
// - Does not detect STOP. The next START always resets the state, so it works without it
//
// No inout: SDA is split into the input sda_i and the "pull low" output sda_low.
`default_nettype none

module i2c_whoami #(
    parameter [6:0] ADDR = 7'h29
) (
    input  wire rst_n,
    input  wire scl,
    input  wire sda_i,
    output reg  sda_low
);

  localparam [7:0] REG_WHO_AM_I = 8'h0F;
  localparam [7:0] REG_SCRATCH  = 8'h10;
  localparam [7:0] WHO_AM_I     = 8'hA5;

  localparam [2:0] S_IDLE     = 3'd0;
  localparam [2:0] S_ADDR     = 3'd1;
  localparam [2:0] S_ADDR_ACK = 3'd2;
  localparam [2:0] S_WDATA    = 3'd3;
  localparam [2:0] S_WACK     = 3'd4;
  localparam [2:0] S_RDATA    = 3'd5;
  localparam [2:0] S_RACK     = 3'd6;

  // ---- START detection (runs on the falling edge of SDA) ----
  // start_tgl toggles on every START. The SCL side compares it with start_ack
  // to know whether a START is still pending
  reg start_tgl;
  reg start_ack;
  always @(negedge sda_i or negedge rst_n) begin
    if (!rst_n) start_tgl <= 1'b0;
    else if (scl) start_tgl <= ~start_tgl;
  end
  wire start_pending = start_tgl ^ start_ack;

  // ---- receive (runs on the rising edge of SCL) ----
  // bitcnt is the number of bits received in the current byte. Back to 0 on the 9th clock (ACK)
  reg [3:0] bitcnt;
  reg [7:0] shreg;
  reg       ack_bit;  // SDA on the 9th clock (the master's ACK/NACK when reading)
  always @(posedge scl or negedge rst_n) begin
    if (!rst_n) begin
      start_ack <= 1'b0;
      bitcnt    <= 4'd0;
      shreg     <= 8'h00;
      ack_bit   <= 1'b1;
    end else if (start_pending) begin
      // the first clock after START = the MSB of the address
      start_ack <= start_tgl;
      shreg     <= {7'h00, sda_i};
      bitcnt    <= 4'd1;
    end else if (bitcnt == 4'd8) begin
      ack_bit <= sda_i;
      bitcnt  <= 4'd0;
    end else begin
      shreg  <= {shreg[6:0], sda_i};
      bitcnt <= bitcnt + 4'd1;
    end
  end

  // ---- state and SDA output (runs on the falling edge of SCL) ----
  reg [2:0] state;
  reg [6:0] tx;  // remaining bits of the byte being sent
  reg [7:0] ptr;
  reg [7:0] scratch;
  reg       rw;
  reg       first;

  // the register ptr points to
  reg [7:0] rdata;
  always @(*) begin
    case (ptr)
      REG_WHO_AM_I: rdata = WHO_AM_I;
      REG_SCRATCH:  rdata = scratch;
      default:      rdata = 8'h00;
    endcase
  end

  always @(negedge scl or negedge rst_n) begin
    if (!rst_n) begin
      state   <= S_IDLE;
      tx      <= 7'h00;
      ptr     <= 8'h00;
      scratch <= 8'h00;
      rw      <= 1'b0;
      first   <= 1'b0;
      sda_low <= 1'b0;
    end else if (start_pending) begin
      // the falling edge right after START. Get ready to receive the address
      state   <= S_ADDR;
      sda_low <= 1'b0;
    end else begin
      case (state)
        S_ADDR: begin
          if (bitcnt == 4'd8) begin
            if (shreg[7:1] == ADDR) begin
              rw      <= shreg[0];
              sda_low <= 1'b1;  // address ACK
              state   <= S_ADDR_ACK;
            end else begin
              state <= S_IDLE;
            end
          end
        end

        S_ADDR_ACK: begin
          if (rw) begin
            tx      <= rdata[6:0];
            sda_low <= ~rdata[7];
            state   <= S_RDATA;
          end else begin
            sda_low <= 1'b0;
            first   <= 1'b1;
            state   <= S_WDATA;
          end
        end

        S_WDATA: begin
          if (bitcnt == 4'd8) begin
            if (first) begin
              ptr   <= shreg;
              first <= 1'b0;
            end else begin
              if (ptr == REG_SCRATCH) scratch <= shreg;
              ptr <= ptr + 8'd1;
            end
            sda_low <= 1'b1;  // data ACK
            state   <= S_WACK;
          end
        end

        S_WACK: begin
          sda_low <= 1'b0;
          state   <= S_WDATA;
        end

        S_RDATA: begin
          if (bitcnt == 4'd8) begin
            sda_low <= 1'b0;  // release for the master's ACK/NACK
            ptr     <= ptr + 8'd1;
            state   <= S_RACK;
          end else begin
            tx      <= {tx[5:0], 1'b0};
            sda_low <= ~tx[6];
          end
        end

        S_RACK: begin
          if (!ack_bit) begin
            tx      <= rdata[6:0];
            sda_low <= ~rdata[7];
            state   <= S_RDATA;
          end else begin
            state <= S_IDLE;
          end
        end

        default: begin
          sda_low <= 1'b0;
        end
      endcase
    end
  end

endmodule

`default_nettype wire
