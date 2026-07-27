`default_nettype none
// wb_engine.v — band writeback: streams the accumulator band to RAM as one
// SPI 0x02 WRITE burst at base_C. Owns the bank read walk while active
// (top-level muxes select it over the sequencer/mem_fetch during wb).
//
// [WB-1] Format follows readout_mode:
//        raw  (mode=1): C-packing — per row 6 bytes: w0.lo, {0,w0[14:8]},
//                       w1.lo, {0,w1[14:8]}, w2.lo, {0,w2[14:8]}  (bit15=0)
//        int4 (mode=0): A/B-packing — per row 2 bytes: {y1,y0}, {4'h0,y2}.
//                       This is EXACTLY the activation-slice input format,
//                       so a band written in int4 is directly consumable as
//                       the next layer's base_A: on-chip layer chaining.
// [WB-2] Pull-model: wb walks the bank itself (wb_rd_en/wb_rd_addr), one
//        row prefetched while the previous row's bytes drain; a byte takes
//        >= 16 clk on the wire while a bank read takes 2, so the S3 8-clk
//        supply budget is met with margin.
// [WB-3] One burst per band: len = 6*R (raw) or 2*R (int4); done asserts
//        only after spi_master retires the burst INCLUDING cs_gap (S5),
//        so "done" is also "safe to issue the next SPI op".
//
// Contract: mem_fetch must be idle for the whole writeback (sequencer
// guarantees: wb only runs from S_RDOUT, never concurrent with fetches).

module wb_engine (
    input  wire        clk,
    input  wire        rst,

    input  wire        start,           // pulse; latches cfg below
    input  wire        mode_raw,        // csr_readout_mode
    input  wire [3:0]  R4,              // 1..8
    input  wire [15:0] base_C,
    output reg         done,            // 1-cycle pulse
    output reg         active,          // high start..done (mux select)

    // bank walk (muxed onto accum bank when active)
    output reg         wb_rd_en,
    output reg  [2:0]  wb_rd_addr,
    input  wire        bk_rd_valid,
    input  wire signed [14:0] rd_w0,
    input  wire signed [14:0] rd_w1,
    input  wire signed [14:0] rd_w2,
    input  wire signed [3:0]  q_y0,     // post requant+relu taps
    input  wire signed [3:0]  q_y1,
    input  wire signed [3:0]  q_y2,

    // spi_master (muxed when active)
    output reg         wb_req,
    output wire        wb_cmd_write,    // constant 1
    output reg  [15:0] wb_addr,
    output reg  [15:0] wb_len,
    output reg  [7:0]  wb_tx_data,
    input  wire        tx_next,
    input  wire        sm_busy
);
    assign wb_cmd_write = 1'b1;

    localparam [2:0] W_IDLE=3'd0, W_RD0=3'd1, W_REQ=3'd2, W_STREAM=3'd3,
                     W_TAIL=3'd4;

    reg [2:0]  st;
    reg        m_raw;
    reg [3:0]  Rl;
    reg [44:0] hold;                    // raw row {w2,w1,w0}
    reg [7:0]  hold_i4 [0:1];           // int4 row pair
    reg [2:0]  bidx;                    // byte index within row
    reg [3:0]  row;                     // current row being drained
    reg        pref_pend;               // bank read in flight for row+1

    wire [2:0] last_b = m_raw ? 3'd5 : 3'd1;

    // byte mux from held row
    reg [7:0] cur_byte;
    always @(*) begin
        if (m_raw) begin
            case (bidx)
            3'd0: cur_byte = hold[7:0];
            3'd1: cur_byte = {1'b0, hold[14:8]};
            3'd2: cur_byte = hold[22:15];
            3'd3: cur_byte = {1'b0, hold[29:23]};
            3'd4: cur_byte = hold[37:30];
            default: cur_byte = {1'b0, hold[44:38]};
            endcase
        end else
            cur_byte = (bidx == 3'd0) ? hold_i4[0] : hold_i4[1];
    end

    task latch_row; begin
        hold       <= {rd_w2, rd_w1, rd_w0};
        hold_i4[0] <= {q_y1, q_y0};
        hold_i4[1] <= {4'h0, q_y2};
    end endtask

    always @(posedge clk) begin
        if (rst) begin
            st <= W_IDLE; done <= 0; active <= 0;
            wb_rd_en <= 0; wb_rd_addr <= 0;
            wb_req <= 0; wb_addr <= 0; wb_len <= 0; wb_tx_data <= 0;
            m_raw <= 0; Rl <= 1; hold <= 0; hold_i4[0] <= 0; hold_i4[1] <= 0;
            bidx <= 0; row <= 0; pref_pend <= 0;
        end else begin
            done <= 0; wb_rd_en <= 0; wb_req <= 0;

            case (st)
            W_IDLE: if (start) begin
                active  <= 1'b1;
                m_raw   <= mode_raw;
                Rl      <= R4;
                wb_addr <= base_C;
                wb_len  <= mode_raw ? (16'd6 * {12'd0, R4})   // 6R (raw, [WB-1])
                                    : (16'd2 * {12'd0, R4});  // 2R (int4)
                row     <= 4'd0; bidx <= 3'd0;
                wb_rd_en   <= 1'b1;              // fetch row 0
                wb_rd_addr <= 3'd0;
                st <= W_RD0;
            end
            W_RD0: if (bk_rd_valid) begin
                latch_row;
                st <= W_REQ;
            end
            W_REQ: begin
                // byte 0 must be on tx_data when req is accepted (S3)
                wb_tx_data <= cur_byte;          // bidx==0
                if (!sm_busy) begin
                    wb_req <= 1'b1;
                    st <= W_STREAM;
                end
            end
            W_STREAM: begin
                wb_tx_data <= cur_byte;          // track mux (within S3 budget)
                if (bk_rd_valid && pref_pend) begin
                    latch_row;                   // next row lands mid-drain
                    pref_pend <= 1'b0;
                end
                if (tx_next) begin
                    if (bidx == last_b) begin
                        bidx <= 3'd0;
                        row  <= row + 4'd1;
                        if (row + 4'd1 == Rl)
                            st <= W_TAIL;        // final byte captured on this pulse
                        else begin
                            // prefetch next row NOW: its first capture pulse
                            // is >= 16*div clk away, the read takes 2 clk,
                            // and the just-captured row's bytes are all
                            // consumed — no clobber window (int4-safe)
                            wb_rd_en   <= 1'b1;
                            wb_rd_addr <= row[2:0] + 3'd1;
                            pref_pend  <= 1'b1;
                        end
                    end else
                        bidx <= bidx + 3'd1;
                end
            end
            W_TAIL: if (!sm_busy) begin          // burst + cs_gap retired (S5)
                active <= 1'b0;
                done   <= 1'b1;
                st <= W_IDLE;
            end
            default: st <= W_IDLE;
            endcase
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) if (!rst && st == W_IDLE && start) begin
        if (R4 == 4'd0 || R4 > 4'd8)
            $display("ERROR wb_engine: R=%0d out of 1..8", R4);
    end
`endif

endmodule

`default_nettype wire