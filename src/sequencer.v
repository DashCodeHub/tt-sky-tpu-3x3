`default_nettype none
// sequencer.v — band-engine control FSM (band_engine_spec v2.1 §4, CD-1..CD-5,
// csr_control_spec, BANK_DEPTH=8). Conducts: array, accum_bank_3col,
// requant_lane x3 + relu, mem_fetch, and the output bus. Consumes
// source-agnostic events (GPIO decoder or mem_fetch, muxed on input_source).
//
// Design tags
// ------------------------------------------------------------------------------
// [SEQ-1] RUN_PASS semantics per source (spec §3.1): GPIO = ONE pass (payload
//         streamed by host); SPI = THE WHOLE BAND autonomously (fetch_vecs
//         per pass, fetch_tile between passes, gated by contract counters).
// [SEQ-2] Band lifecycle: first LOAD_TILE while idle pulses band_start
//         (mem_fetch pointer reload), zeroes kb and ovf_agg, sets band_active;
//         BAND_DONE clears band_active (+ auto STATUS frame, CD-4).
// [SEQ-3] The streaming plane is the MOST-RECENTLY-LOADED plane; loads always
//         target its complement. No flip logic; reusing one tile for several
//         passes is legal for free (just don't LOAD between them).
// [SEQ-4] Contract counters are the hardware embodiment of §5 (grants HELD,
//         never errored): since_last_use[p] (4b saturating, RELOAD_GAP=11)
//         gates the first w_load_en of a chain burst; use_after_load[p]
//         (3b saturating, USE_GAP=3) gates STREAM entry.
// [SEQ-5] CD-1: wr_ptr_rst pulses on EVERY STREAM entry; ovf_agg |= bank
//         stickies continuously (they are sticky until that pulse clears
//         them, so a continuous OR loses nothing and needs no sampling
//         choreography). Cleared at band start.
// [SEQ-6] DRAIN = RESULT_LAT(c=2) + 1 bank write = 14 unstalled cycles after
//         the last vector cycle. stall is tied 0 by this sequencer (bubbles
//         do all pacing; D8 root clock gate is a chip-level power feature).
// [SEQ-7] Weight rows are buffered 3-deep, then driven as 3 CONSECUTIVE
//         chain cycles once the reload grant fires — one gate for the whole
//         burst, matching the +11 contract's "reload may start" wording.
//
// HOST CONTRACT (firmware): H1 LOAD_TILE precedes the first RUN_PASS of a
// band. H2 In GPIO mode, send one RUN_PASS per pass (Nb total). H3 Commands
// only when STATUS.state says the FSM can take them (count-deterministic;
// the FSM ignores, never errors). H4 READ_BAND only after BAND_DONE.

module sequencer (
    input  wire        clk,
    input  wire        rst,

    // ---- CSRs (from cfg_decoder) ----
    input  wire        csr_a_signed,
    input  wire        csr_w_signed,
    input  wire [2:0]  csr_row_pe_en,
    input  wire [2:0]  csr_col_pe_en,
    input  wire [2:0]  csr_shift_amt,
    input  wire        csr_relu_en,
    input  wire        csr_readout_mode,   // 0 int4, 1 raw
    input  wire        csr_wb_en,          // READ_BAND -> SPI writeback (WB-1)
    input  wire        csr_input_source,   // 0 GPIO, 1 SPI
    input  wire [5:0]  csr_R,              // 1..8
    input  wire [6:0]  csr_Nb,

    // ---- command pulses (from cfg_decoder) ----
    input  wire        cmd_load_tile,
    input  wire        cmd_run_pass,
    input  wire        cmd_read_band,
    input  wire        cmd_status,
    input  wire        cfg_busy,

    // ---- source-agnostic events: GPIO decoder ----
    input  wire [11:0] g_wrow_data,
    input  wire        g_wrow_valid,
    input  wire [11:0] g_vec_data,
    input  wire        g_vec_valid,
    // ---- source-agnostic events: mem_fetch ----
    input  wire [11:0] m_wrow_data,
    input  wire        m_wrow_valid,
    input  wire [11:0] m_vec_data,
    input  wire        m_vec_valid,

    // ---- mem_fetch control ----
    output reg         band_start,
    output reg         fetch_tile,
    output reg         fetch_vecs,
    input  wire        mf_busy,
    input  wire        tile_done,
    input  wire        vecs_done,

    // ---- array drive ----
    output reg  signed [3:0] a_west0,
    output reg  signed [3:0] a_west1,
    output reg  signed [3:0] a_west2,
    output reg         valid_west,          // fans to all three valid_west*
    output reg         plane_west,          // fans to all three plane_west*
    output reg  signed [3:0] w_north0,
    output reg  signed [3:0] w_north1,
    output reg  signed [3:0] w_north2,
    output reg         w_load_en,
    output reg         w_fill_sel,
    output wire        stall,               // [SEQ-6] tied 0

    // ---- bank control / status ----
    output reg         bk_wr_en,
    output reg         bk_first_pass,
    output reg         bk_wr_ptr_rst,
    output reg         bk_rd_en,
    output reg  [2:0]  bk_rd_addr,
    input  wire        bk_rd_valid,
    input  wire [2:0]  bk_ovf_sticky,       // {ovf2,ovf1,ovf0}

    // ---- readout datapath taps ----
    input  wire signed [14:0] rd_w0,        // bank rd_data (raw mode source)
    input  wire signed [14:0] rd_w1,
    input  wire signed [14:0] rd_w2,
    input  wire signed [3:0]  q_y0,         // post requant+relu (int4 mode)
    input  wire signed [3:0]  q_y1,
    input  wire signed [3:0]  q_y2,

    // ---- writeback engine ----
    output reg         wb_start,
    input  wire        wb_done,

    // ---- output bus (12-bit beats) ----
    output reg  [11:0] out_beat,
    output reg         out_valid
);
    assign stall = 1'b0;                    // [SEQ-6]

    localparam [2:0] S_IDLE=3'd0, S_CFGW=3'd1, S_LOAD=3'd2, S_WUSE=3'd3,
                     S_STRM=3'd4, S_DRAIN=3'd5, S_BDONE=3'd6, S_RDOUT=3'd7;
    localparam [3:0] RELOAD_GAP = 4'd11;    // §5, boundary-verified
    localparam [2:0] USE_GAP    = 3'd3;
    localparam [3:0] DRAIN_N    = 4'd14;    // [SEQ-6]

    reg [2:0]  state;
    reg        band_active;
    reg        active_plane;                // [SEQ-3] = last loaded plane
    reg [6:0]  kb;
    reg [3:0]  vec_cnt;
    reg [3:0]  drain_cnt;
    reg [2:0]  ovf_agg;
    reg        band_done_sticky;
    reg        spi_band;                    // [SEQ-1] this RUN_PASS runs whole band
    reg        pend_run;                    // GPIO RUN_PASS seen while draining etc.

    // ---- event mux ----
    wire [11:0] ev_wrow_d = csr_input_source ? m_wrow_data  : g_wrow_data;
    wire        ev_wrow_v = csr_input_source ? m_wrow_valid : g_wrow_valid;
    wire [11:0] ev_vec_d  = csr_input_source ? m_vec_data   : g_vec_data;
    wire        ev_vec_v  = csr_input_source ? m_vec_valid  : g_vec_valid;

    // ---- [SEQ-4] contract counters ----
    reg [3:0] since_use  [0:1];             // saturating at 15
    reg [2:0] after_load [0:1];             // saturating at 7
    wire       load_tgt      = band_active ? ~active_plane : 1'b0; // first load -> plane 0
    wire       reload_elig   = (since_use[load_tgt]  >= RELOAD_GAP);
    wire       use_elig      = (after_load[active_plane] >= USE_GAP);

    // ---- [SEQ-7] weight-row buffer + chain burst engine (runs in parallel
    //      with the main FSM so mid-STREAM GPIO loads work: double-buffer
    //      loading during streaming is array-verified behavior) ----
    reg [11:0] row_buf [0:2];
    reg [1:0]  rows_have;                   // 0..3 collected
    reg [1:0]  chain_idx;                   // 0..3 driving
    reg        chain_run;
    reg        load_pending;                // a LOAD command owns the buffer
    wire       chain_done_now = chain_run && (chain_idx == 2'd2);

    // ---- readout raw packer ----
    reg [44:0] raw_hold;
    reg [1:0]  raw_beat;
    reg        raw_have;
    reg        rd_pend;                     // a read is in flight

    // ---- pending SPI tile prefetch ----
    reg        pend_tile;

    integer p;

    wire [3:0]  R4 = csr_R[3:0];        // 1..8 fits 4 bits (contract-checked)
    wire [11:0] status_frame = {2'b00, band_done_sticky, use_elig, reload_elig,
                                ovf_agg, active_plane, state};
    reg  [3:0]  rd_row4;
    reg         pend_vecs;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE; band_active <= 0; active_plane <= 0;
            kb <= 0; vec_cnt <= 0; drain_cnt <= 0; ovf_agg <= 0;
            band_done_sticky <= 0; spi_band <= 0; pend_run <= 0;
            band_start <= 0; fetch_tile <= 0; fetch_vecs <= 0;
            a_west0 <= 0; a_west1 <= 0; a_west2 <= 0;
            valid_west <= 0; plane_west <= 0;
            w_north0 <= 0; w_north1 <= 0; w_north2 <= 0;
            w_load_en <= 0; w_fill_sel <= 0;
            bk_wr_en <= 0; bk_first_pass <= 0; bk_wr_ptr_rst <= 0;
            bk_rd_en <= 0; bk_rd_addr <= 0;
            out_beat <= 0; out_valid <= 0; wb_start <= 0;
            since_use[0] <= 4'hF; since_use[1] <= 4'hF;
            after_load[0] <= 3'h7; after_load[1] <= 3'h7;
            rows_have <= 0; chain_idx <= 0; chain_run <= 0; load_pending <= 0;
            raw_hold <= 0; raw_beat <= 0; raw_have <= 0; rd_row4 <= 0; rd_pend <= 0;
            pend_tile <= 0; pend_vecs <= 0;
        end else begin
            // ---- defaults (pulses) ----
            band_start <= 0; fetch_tile <= 0; fetch_vecs <= 0;
            valid_west <= 0; w_load_en <= 0; bk_wr_ptr_rst <= 0;
            bk_rd_en <= 0; out_valid <= 0; wb_start <= 0;

            // ---- contract counters tick every cycle [SEQ-4] ----
            for (p = 0; p < 2; p = p + 1) begin
                if (since_use[p]  != 4'hF) since_use[p]  <= since_use[p]  + 4'd1;
                if (after_load[p] != 3'h7) after_load[p] <= after_load[p] + 3'd1;
            end

            // ---- [SEQ-5] ovf aggregation ----
            ovf_agg <= ovf_agg | bk_ovf_sticky;

            // ---- weight-row collection (any state; buffer owns 3 slots) ----
            if (ev_wrow_v && rows_have != 2'd3) begin
                row_buf[rows_have] <= ev_wrow_d;
                rows_have <= rows_have + 2'd1;
            end
            // chain burst: start when 3 rows buffered AND grant fires [SEQ-7]
            if (!chain_run && rows_have == 2'd3 && reload_elig && load_pending) begin
                chain_run <= 1'b1;
                chain_idx <= 2'd0;
            end
            if (chain_run) begin
                w_load_en  <= 1'b1;
                w_fill_sel <= load_tgt;
                w_north0   <= row_buf[chain_idx][3:0];
                w_north1   <= row_buf[chain_idx][7:4];
                w_north2   <= row_buf[chain_idx][11:8];
                if (chain_idx == 2'd2) begin
                    chain_run    <= 1'b0;
                    rows_have    <= 2'd0;
                    load_pending <= 1'b0;
                    active_plane <= load_tgt;            // [SEQ-3]
                    after_load[load_tgt] <= 3'd0;
                end else
                    chain_idx <= chain_idx + 2'd1;
            end

            // ---- main FSM ----
            case (state)
            S_IDLE: begin
                bk_wr_en <= 1'b0;
                if (cfg_busy) state <= S_CFGW;
                else if (cmd_load_tile) begin
                    if (!band_active) begin              // [SEQ-2]
                        band_active <= 1'b1; band_start <= 1'b1;
                        kb <= 7'd0; ovf_agg <= 3'd0;
                        band_done_sticky <= 1'b0;
                    end
                    load_pending <= 1'b1;
                    // SPI: defer via pend_tile so band_start's pointer reload
                    // lands BEFORE the fetch samples b_ptr (MD1 race fix)
                    if (csr_input_source) pend_tile <= 1'b1;
                    state <= S_LOAD;
                end else if (cmd_run_pass && band_active) begin
                    spi_band <= csr_input_source;        // [SEQ-1]
                    band_done_sticky <= 1'b0;
                    state <= S_WUSE;
                end else if (cmd_read_band) begin
                    rd_row4 <= 4'd0; rd_pend <= 1'b0; raw_have <= 1'b0;
                    if (csr_wb_en) wb_start <= 1'b1;   // [WB-2] hand off
                    state <= S_RDOUT;
                end else if (cmd_status) begin
                    out_beat <= status_frame; out_valid <= 1'b1;
                end
            end

            S_CFGW: if (!cfg_busy) state <= S_IDLE;

            S_LOAD: begin
                // wait for the chain burst to complete (grant may hold it)
                if (chain_done_now)
                    state <= spi_band ? S_WUSE : S_IDLE;   // [SEQ-1] band loop
            end

            S_WUSE: if (use_elig) begin
                // STREAM entry: CD-1 pulse + pass setup
                bk_wr_ptr_rst <= 1'b1;                   // [SEQ-5]
                bk_wr_en      <= 1'b1;
                bk_first_pass <= (kb == 7'd0);
                vec_cnt       <= 4'd0;
                if (csr_input_source) begin
                    if (!mf_busy) fetch_vecs <= 1'b1;
                    else          pend_vecs  <= 1'b1;
                end
                state <= S_STRM;
            end

            S_STRM: begin
                bk_wr_en <= 1'b1;
                if (ev_vec_v) begin
                    a_west0 <= ev_vec_d[3:0];
                    a_west1 <= ev_vec_d[7:4];
                    a_west2 <= ev_vec_d[11:8];
                    valid_west <= 1'b1;
                    plane_west <= active_plane;
                    since_use[active_plane] <= 4'd0;     // [SEQ-4]
                    vec_cnt <= vec_cnt + 4'd1;
                    if (vec_cnt + 4'd1 == R4) begin
                        drain_cnt <= 4'd0;
                        state <= S_DRAIN;
                    end
                end
            end

            S_DRAIN: begin
                bk_wr_en <= 1'b1;                        // writes still landing
                drain_cnt <= drain_cnt + 4'd1;
                if (drain_cnt == DRAIN_N - 4'd1) begin   // [SEQ-6]
                    if (kb + 7'd1 < csr_Nb) begin
                        kb <= kb + 7'd1;
                        if (spi_band) begin
                            // [SEQ-1] autonomous: prefetch next tile, then loop
                            load_pending <= 1'b1;
                            pend_tile    <= 1'b1;
                            state <= S_LOAD;
                        end else
                            state <= S_IDLE;             // await next RUN_PASS (H2)
                    end else begin
                        band_active <= 1'b0;
                        band_done_sticky <= 1'b1;
                        out_beat <= {2'b00, 1'b1, use_elig, reload_elig,
                                     ovf_agg | bk_ovf_sticky, active_plane, S_BDONE};
                        out_valid <= 1'b1;               // CD-4 auto STATUS
                        state <= S_BDONE;
                    end
                end
            end

            S_BDONE: state <= S_IDLE;

            S_RDOUT: if (csr_wb_en) begin
                if (wb_done) state <= S_IDLE;          // wb_engine owns the walk
            end else begin
                if (!rd_pend && !raw_have) begin
                    bk_rd_en   <= 1'b1;
                    bk_rd_addr <= rd_row4[2:0];
                    rd_pend    <= 1'b1;
                end else if (rd_pend && bk_rd_valid) begin
                    rd_pend <= 1'b0;
                    if (!csr_readout_mode) begin
                        out_beat  <= {q_y2, q_y1, q_y0};
                        out_valid <= 1'b1;
                        if (rd_row4 + 4'd1 == R4) state <= S_IDLE;
                        else rd_row4 <= rd_row4 + 4'd1;
                    end else begin
                        raw_hold <= {rd_w2, rd_w1, rd_w0};
                        raw_beat <= 2'd0;
                        raw_have <= 1'b1;
                    end
                end else if (raw_have) begin
                    out_valid <= 1'b1;
                    case (raw_beat)
                    2'd0: out_beat <= raw_hold[11:0];
                    2'd1: out_beat <= raw_hold[23:12];
                    2'd2: out_beat <= raw_hold[35:24];
                    2'd3: out_beat <= {ovf_agg, raw_hold[44:36]};
                    endcase
                    if (raw_beat == 2'd3) begin
                        raw_have <= 1'b0;
                        if (rd_row4 + 4'd1 == R4) state <= S_IDLE;
                        else rd_row4 <= rd_row4 + 4'd1;
                    end else
                        raw_beat <= raw_beat + 2'd1;
                end
            end

            default: state <= S_IDLE;
            endcase

            // mid-band LOAD_TILE command (GPIO mid-stream or SPI prefetch):
            // accepted without leaving the current state (double-buffer load
            // during streaming is array-verified behavior)
            if (cmd_load_tile && band_active && state != S_IDLE) begin
                load_pending <= 1'b1;
                if (csr_input_source) pend_tile <= 1'b1;
            end
            // deferred mem_fetch requests: issue when the bus frees
            if (pend_tile && !mf_busy && !fetch_tile && !fetch_vecs) begin
                fetch_tile <= 1'b1;
                pend_tile  <= 1'b0;
            end
            if (pend_vecs && !mf_busy && !fetch_tile && !fetch_vecs && state == S_STRM) begin
                fetch_vecs <= 1'b1;
                pend_vecs  <= 1'b0;
            end
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) if (!rst) begin
        if (csr_R == 6'd0 || csr_R > 6'd8)
            $display("ERROR sequencer: R=%0d out of contract 1..8", csr_R);
        if (csr_row_pe_en == 3'b101 || csr_row_pe_en == 3'b010 ||
            csr_col_pe_en == 3'b101 || csr_col_pe_en == 3'b010)
            $display("ERROR sequencer: non-thermometer enable mask");
    end
`endif

endmodule

`default_nettype wire