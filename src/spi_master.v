`default_nettype none
// spi_master.v
// SPI mode-0 burst master for the RP2040 spi-ram-emu (23LC512-like) slave,
// per spi_ram_emu_facts.md. Designed as the memory port for mem_fetch /
// writeback (Section 10 steps 3/5); config inputs come from the future CSR
// block.
//
// Protocol implemented (facts §1-§3):
//   READ  0x03 : cmd + 16b addr, data immediately        (bring-up compare)
//   FREAD 0x0B : cmd + 16b addr + 8 dummy SCK, then data (default: flat
//                sys/8 slave ceiling regardless of alignment)
//   WRITE 0x02 : cmd + 16b addr + data bytes
//   All MSB first. Burst length arbitrary; CONTRACT: addr+len <= 65536
//   (no wrap past 0xFFFF — UB at the slave; sim-only check below).
//
// Design tags
// ------------------------------------------------------------------------------
// [SM1] SCK = clk / (2*cfg_div), cfg_div >= 1. Full-rate SCK (nanoV's
//       `!clk && en` trick) deliberately NOT used: our chip clock (50-111
//       MHz) exceeds the emu's 8-15 MHz ceilings; nanoV ran at 12-14 MHz.
//       Mode 0: SCK idles low, MOSI launched on falling edges (and at CS
//       assert), MISO sampled on rising edges.
// [SM2] MISO metastability/margin: captured continuously on the chip
//       clock's NEGEDGE (miso_r), giving a half-cycle round-trip budget.
//       Technique from MichaelBell/nanoV top.v (Apache-2.0), attributed.
// [SM3] cfg_cs_gap: minimum clk cycles CS stays high after a transaction
//       before busy releases — covers the emu's unspecified core1 recovery
//       time (facts §5, [MEASURE]); bench-tune the CSR default down later.
// [SM4] Streaming interfaces sized for mem_fetch: one rx_valid pulse per
//       received byte; write side is load-then-request (tx_next pulses when
//       a byte is taken into the shifter, provider then has >= 16*cfg_div-2
//       clk to present the next byte — at cfg_div=1 that is >= 14 clk).
// [SM5] Long-burst bias (nanoV usage lesson): CS toggles are the expensive
//       thing (each pays cfg_cs_gap + 3-byte header + dummies); callers
//       should fetch in the longest legal bursts.
//
// USAGE CONTRACT (enforced by mem_fetch / CSR block)
// ------------------------------------------------------------------------------
// S1. cfg_* are static while busy=1 (change only between transactions).
// S2. req is sampled only when busy=0; assert req for one cycle with
//     cmd_write/addr/len valid. len >= 1 bytes; addr+len <= 65536.
// S3. Writes: tx_data must hold byte 0 when req is accepted, and be updated
//     to the next byte within 8 clk of each tx_next pulse ([SM4] budget).
// S4. Reads: rx_data is valid for exactly the rx_valid cycle plus until the
//     next byte completes; consumers should capture on rx_valid.
// S5. busy covers the whole transaction INCLUDING the trailing cs_gap;
//     "!busy" alone is the safe-to-issue condition.

module spi_master (
    input  wire        clk,
    input  wire        rst,

    // static config (future CSRs)
    input  wire [7:0]  cfg_div,        // [SM1] SCK = clk/(2*div); >=1
    input  wire [15:0] cfg_cs_gap,     // [SM3] post-CS-high recovery, clk cycles
    input  wire        cfg_fast_read,  // 1: 0x0B (default), 0: 0x03

    // command interface (mem_fetch side)
    input  wire        req,
    input  wire        cmd_write,      // 0 = read, 1 = write
    input  wire [15:0] addr,
    input  wire [15:0] len,            // bytes, >= 1
    output reg         busy,

    // read byte stream
    output reg  [7:0]  rx_data,
    output reg         rx_valid,

    // write byte stream (load-then-request, [SM4])
    input  wire [7:0]  tx_data,
    output reg         tx_next,

    // pads (official TT mapping at wrapper: uio0=CS uio1=MOSI uio2=MISO uio3=SCK)
    output reg         spi_cs_n,
    output reg         spi_sck,
    output reg         spi_mosi,
    input  wire        spi_miso
);

    localparam [2:0] ST_IDLE  = 3'd0,
                     ST_SETUP = 3'd1,   // CS low, SCK low, first bit on MOSI
                     ST_HDR   = 3'd2,   // cmd + addr, 24 bits
                     ST_DUMMY = 3'd3,   // 8 SCK, fast read only
                     ST_DATA  = 3'd4,   // payload bytes
                     ST_TAIL  = 3'd5,   // final SCK-low half period
                     ST_GAP   = 3'd6;   // CS high, count cfg_cs_gap

    reg [2:0]  state;
    reg [7:0]  div_cnt;
    reg        is_write, fast;
    reg [23:0] hdr_shift;
    reg [4:0]  bit_cnt;            // rising edges within current phase byte
    reg [7:0]  tx_shift, rx_shift;
    reg [16:0] bytes_left;
    reg [15:0] gap_cnt;
    reg        miso_r;

    wire tick = (div_cnt == cfg_div - 8'd1);   // one half-period elapsed

    // [SM2] negedge capture of the return path (nanoV technique, Apache-2.0)
    always @(negedge clk) miso_r <= spi_miso;

    always @(posedge clk) begin
        if (rst) begin
            state    <= ST_IDLE;
            busy     <= 1'b0;
            spi_cs_n <= 1'b1;
            spi_sck  <= 1'b0;
            spi_mosi <= 1'b0;
            rx_valid <= 1'b0;
            tx_next  <= 1'b0;
            div_cnt  <= 8'd0;
            gap_cnt  <= 16'd0;
            rx_data  <= 8'd0;
            rx_shift <= 8'd0;
            tx_shift <= 8'd0;
            hdr_shift<= 24'd0;
            bit_cnt  <= 5'd0;
            bytes_left <= 17'd0;
            is_write <= 1'b0;
            fast     <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            tx_next  <= 1'b0;

            case (state)
            ST_IDLE: begin
                spi_cs_n <= 1'b1;
                spi_sck  <= 1'b0;
                div_cnt  <= 8'd0;
                if (req) begin
                    busy      <= 1'b1;
                    is_write  <= cmd_write;
                    fast      <= cfg_fast_read;
                    hdr_shift <= {cmd_write ? 8'h02
                                            : (cfg_fast_read ? 8'h0B : 8'h03),
                                  addr};
                    bytes_left <= {1'b0, len};
                    bit_cnt   <= 5'd0;
                    state     <= ST_SETUP;
`ifndef SYNTHESIS
                    if (len == 16'd0)
                        $display("ERROR spi_master: len must be >= 1");
                    if ({1'b0, addr} + {1'b0, len} > 17'd65536)
                        $display("ERROR spi_master: burst crosses 0xFFFF (UB)");
`endif
                end
            end

            ST_SETUP: begin
                // assert CS with SCK low; place first header bit on MOSI;
                // hold one half period of setup before the first rising edge
                spi_cs_n <= 1'b0;
                spi_mosi <= hdr_shift[23];
                if (div_cnt == cfg_div - 8'd1) begin
                    div_cnt <= 8'd0;
                    state   <= ST_HDR;
                end else
                    div_cnt <= div_cnt + 8'd1;
            end

            ST_HDR: begin
                if (tick) begin
                    div_cnt <= 8'd0;
                    if (!spi_sck) begin
                        spi_sck <= 1'b1;               // rising edge: slave samples
                        bit_cnt <= bit_cnt + 5'd1;
                    end else begin
                        spi_sck <= 1'b0;               // falling edge: launch next
                        if (bit_cnt == 5'd24) begin
                            bit_cnt <= 5'd0;
                            if (is_write) begin
                                tx_shift <= tx_data;   // [SM4] load then request
                                tx_next  <= 1'b1;
                                spi_mosi <= tx_data[7];
                                state    <= ST_DATA;
                            end else begin
                                spi_mosi <= 1'b0;
                                state    <= fast ? ST_DUMMY : ST_DATA;
                            end
                        end else begin
                            hdr_shift <= {hdr_shift[22:0], 1'b0};
                            spi_mosi  <= hdr_shift[22];
                        end
                    end
                end else
                    div_cnt <= div_cnt + 8'd1;
            end

            ST_DUMMY: begin                             // 8 SCK, MOSI low
                spi_mosi <= 1'b0;
                if (tick) begin
                    div_cnt <= 8'd0;
                    if (!spi_sck) begin
                        spi_sck <= 1'b1;
                        bit_cnt <= bit_cnt + 5'd1;
                    end else begin
                        spi_sck <= 1'b0;
                        if (bit_cnt == 5'd8) begin
                            bit_cnt <= 5'd0;
                            state   <= ST_DATA;
                        end
                    end
                end else
                    div_cnt <= div_cnt + 8'd1;
            end

            ST_DATA: begin
                if (tick) begin
                    div_cnt <= 8'd0;
                    if (!spi_sck) begin
                        spi_sck <= 1'b1;               // rising edge
                        bit_cnt <= bit_cnt + 5'd1;
                        if (!is_write) begin
                            rx_shift <= {rx_shift[6:0], miso_r};
                            if (bit_cnt == 5'd7) begin // 8th bit this edge
                                rx_data  <= {rx_shift[6:0], miso_r};
                                rx_valid <= 1'b1;
                            end
                        end
                    end else begin
                        spi_sck <= 1'b0;               // falling edge
                        if (bit_cnt == 5'd8) begin
                            bit_cnt <= 5'd0;
                            if (bytes_left == 17'd1) begin
                                spi_mosi <= 1'b0;
                                state    <= ST_TAIL;
                            end else begin
                                bytes_left <= bytes_left - 17'd1;
                                if (is_write) begin
                                    tx_shift <= tx_data;
                                    tx_next  <= 1'b1;
                                    spi_mosi <= tx_data[7];
                                end
                            end
                        end else if (is_write) begin
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            spi_mosi <= tx_shift[6];
                        end
                    end
                end else
                    div_cnt <= div_cnt + 8'd1;
            end

            ST_TAIL: begin                              // half period SCK low
                if (tick) begin
                    div_cnt  <= 8'd0;
                    spi_cs_n <= 1'b1;
                    spi_mosi <= 1'b0;
                    gap_cnt  <= cfg_cs_gap;             // [SM3]
                    state    <= ST_GAP;
                end else
                    div_cnt <= div_cnt + 8'd1;
            end

            ST_GAP: begin
                if (gap_cnt == 16'd0) begin
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end else
                    gap_cnt <= gap_cnt - 16'd1;
            end

            default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire