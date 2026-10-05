// i2c_whoami: a minimal I2C slave
//
//   7-bit address ADDR (default 0x29)
//   0x0F WHO_AM_I  = 0xA5 (RO)
//   0x10 SCRATCH         (RW)
//   Other addresses read 0x00; writes to them are ignored
//
// The first byte of a write transaction is the register pointer. The pointer
// auto-increments with each data byte.
//
// SCL/SDA are synchronized to the system clock clk with two flip-flops and oversampled.
// No inout: SDA is split into the input sda_i and the "pull low" output sda_low.
// The wired-AND on the bus is resolved by the simulator (Rust side).
`default_nettype none

module i2c_whoami #(
    parameter [6:0] ADDR = 7'h29
) (
    input  wire clk,
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

  // two-stage synchronizer + one stage of the previous value
  reg [2:0] scl_s;
  reg [2:0] sda_s;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      scl_s <= 3'b111;
      sda_s <= 3'b111;
    end else begin
      scl_s <= {scl_s[1:0], scl};
      sda_s <= {sda_s[1:0], sda_i};
    end
  end

  wire scl_q = scl_s[1];
  wire scl_p = scl_s[2];
  wire sda_q = sda_s[1];
  wire sda_p = sda_s[2];

  wire scl_rise = scl_q & ~scl_p;
  wire scl_fall = ~scl_q & scl_p;
  wire start    = scl_q & scl_p & sda_p & ~sda_q;
  wire stop     = scl_q & scl_p & ~sda_p & sda_q;

  reg [2:0] state;
  reg [3:0] bitcnt;
  reg [7:0] shreg;
  reg [6:0] tx;  // remaining bits of the byte being sent
  reg [7:0] ptr;
  reg [7:0] scratch;
  reg       rw;
  reg       first;
  reg       master_ack;

  // the register ptr points to
  reg [7:0] rdata;
  always @(*) begin
    case (ptr)
      REG_WHO_AM_I: rdata = WHO_AM_I;
      REG_SCRATCH:  rdata = scratch;
      default:      rdata = 8'h00;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= S_IDLE;
      bitcnt     <= 4'd0;
      shreg      <= 8'h00;
      tx         <= 7'h00;
      ptr        <= 8'h00;
      scratch    <= 8'h00;
      rw         <= 1'b0;
      first      <= 1'b0;
      master_ack <= 1'b0;
      sda_low    <= 1'b0;
    end else if (start) begin
      // START / repeated START
      state   <= S_ADDR;
      bitcnt  <= 4'd0;
      sda_low <= 1'b0;
    end else if (stop) begin
      state   <= S_IDLE;
      sda_low <= 1'b0;
    end else begin
      case (state)
        S_ADDR: begin
          if (scl_rise) begin
            shreg  <= {shreg[6:0], sda_q};
            bitcnt <= bitcnt + 4'd1;
          end else if (scl_fall && bitcnt == 4'd8) begin
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
          if (scl_fall) begin
            bitcnt <= 4'd0;
            if (rw) begin
              tx      <= rdata[6:0];
              sda_low <= ~rdata[7];
              bitcnt  <= 4'd1;
              state   <= S_RDATA;
            end else begin
              sda_low <= 1'b0;
              first   <= 1'b1;
              state   <= S_WDATA;
            end
          end
        end

        S_WDATA: begin
          if (scl_rise) begin
            shreg  <= {shreg[6:0], sda_q};
            bitcnt <= bitcnt + 4'd1;
          end else if (scl_fall && bitcnt == 4'd8) begin
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
          if (scl_fall) begin
            sda_low <= 1'b0;
            bitcnt  <= 4'd0;
            state   <= S_WDATA;
          end
        end

        S_RDATA: begin
          if (scl_fall) begin
            if (bitcnt == 4'd8) begin
              sda_low <= 1'b0;  // release for the master's ACK/NACK
              ptr     <= ptr + 8'd1;
              state   <= S_RACK;
            end else begin
              tx      <= {tx[5:0], 1'b0};
              sda_low <= ~tx[6];
              bitcnt  <= bitcnt + 4'd1;
            end
          end
        end

        S_RACK: begin
          if (scl_rise) begin
            master_ack <= ~sda_q;
          end else if (scl_fall) begin
            if (master_ack) begin
              tx      <= rdata[6:0];
              sda_low <= ~rdata[7];
              bitcnt  <= 4'd1;
              state   <= S_RDATA;
            end else begin
              state <= S_IDLE;
            end
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
