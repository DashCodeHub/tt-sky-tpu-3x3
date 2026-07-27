`default_nettype none
// chip_core.v — Project Sky TPU integration core + D7 output stage.
// Everything TT-agnostic lives here: cfg_decoder, sequencer, wb_engine,
// mem_fetch, spi_master, array, bank, requant+relu, the wb_active phase
// muxes, and the D7 OUTPUT SERIALIZER stage:
//
// [D7-1] frame[11:0] carries one beat per frame_valid cycle, REGISTERED,
//        and is driven 0 between beats (quiet-bus idle). Dual readout modes
//        per the D7 requirement: int4 = {y2,y1,y0} one frame/row; raw =
//        4 frames/row per CD-3 ({ovf_agg, w2, w1, w0} LE 12b chunks).
// [D7-2] frame_valid is exposed for harnesses (cocotb/FPGA bring-up); the
//        TT pinout intentionally drops it — pin-level hosts use the
//        measured count-deterministic offsets instead (READ_BAND int4:
//        header+6 then 3/row; STATUS: header+3; +includes this stage's
//        1-cycle latency).

module chip_core (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  beat_in,
    output reg  [11:0] frame,
    output reg         frame_valid,
    output wire        spi_cs_n,
    output wire        spi_sck,
    output wire        spi_mosi,
    input  wire        spi_miso
);
    // ---------------- decoder ----------------
    wire        a_signed, w_signed, relu_en, readout_mode, input_source;
    wire        fastrd, wb_en;
    wire [2:0]  row_en, col_en, shift_amt;
    wire [5:0]  R;
    wire [6:0]  Nb;
    wire [7:0]  spi_div;
    wire [15:0] spi_cs_gap, base_A, base_B, base_C;
    wire        c_load, c_run, c_read, c_stat, cfg_busy;
    wire [11:0] g_wrow_d, g_vec_d;
    wire        g_wrow_v, g_vec_v;

    cfg_decoder dec (
        .clk(clk), .rst(rst), .in_beat(beat_in),
        .csr_a_signed(a_signed), .csr_w_signed(w_signed),
        .csr_relu_en(relu_en), .csr_readout_mode(readout_mode),
        .csr_input_source(input_source), .csr_spi_fast_read(fastrd),
        .csr_wb_en(wb_en),
        .csr_row_pe_en(row_en), .csr_col_pe_en(col_en),
        .csr_shift_amt(shift_amt), .csr_R(R), .csr_Nb(Nb),
        .csr_spi_div(spi_div), .csr_spi_cs_gap(spi_cs_gap),
        .csr_base_A(base_A), .csr_base_B(base_B), .csr_base_C(base_C),
        .cmd_load_tile(c_load), .cmd_run_pass(c_run),
        .cmd_read_band(c_read), .cmd_status(c_stat), .cfg_busy(cfg_busy),
        .g_wrow_data(g_wrow_d), .g_wrow_valid(g_wrow_v),
        .g_vec_data(g_vec_d), .g_vec_valid(g_vec_v)
    );

    // ---------------- mem_fetch + spi_master ----------------
    wire        band_start, fetch_tile, fetch_vecs, mf_busy, tile_done, vecs_done;
    wire [11:0] m_wrow_d, m_vec_d;
    wire        m_wrow_v, m_vec_v;
    wire        mf_req; wire [15:0] mf_addr, mf_len;
    wire        sm_busy; wire [7:0] rx_data; wire rx_valid;
    wire        tx_next;

    wire        wb_start, wb_done, wb_active, wb_req, wb_cmd_write;
    wire [15:0] wb_addr, wb_len;
    wire [7:0]  wb_tx;
    wire        wb_rd_en; wire [2:0] wb_rd_addr;

    mem_fetch mf (
        .clk(clk), .rst(rst),
        .base_A(base_A), .base_B(base_B), .R(R),
        .band_start(band_start), .fetch_tile(fetch_tile), .fetch_vecs(fetch_vecs),
        .mf_busy(mf_busy), .tile_done(tile_done), .vecs_done(vecs_done),
        .wrow_data(m_wrow_d), .wrow_valid(m_wrow_v),
        .vec_data(m_vec_d), .vec_valid(m_vec_v),
        .sm_req(mf_req), .sm_addr(mf_addr), .sm_len(mf_len),
        .sm_busy(sm_busy), .rx_data(rx_data), .rx_valid(rx_valid)
    );

    // [TT-2] wb phase mux
    spi_master sm (
        .clk(clk), .rst(rst),
        .cfg_div(spi_div), .cfg_cs_gap(spi_cs_gap), .cfg_fast_read(fastrd),
        .req(wb_active ? wb_req : mf_req),
        .cmd_write(wb_active ? wb_cmd_write : 1'b0),
        .addr(wb_active ? wb_addr : mf_addr),
        .len(wb_active ? wb_len : mf_len),
        .busy(sm_busy), .rx_data(rx_data), .rx_valid(rx_valid),
        .tx_data(wb_tx), .tx_next(tx_next),
        .spi_cs_n(spi_cs_n), .spi_sck(spi_sck),
        .spi_mosi(spi_mosi), .spi_miso(spi_miso)
    );

    // ---------------- sequencer ----------------
    wire signed [3:0] aw0, aw1, aw2, wn0, wn1, wn2;
    wire valid_w, plane_w, w_load_en, w_fill_sel, stall;
    wire bk_wr_en, bk_fp, bk_wpr, seq_rd_en;
    wire [2:0] seq_rd_addr;
    wire bk_rd_valid;
    wire [2:0] ovf_sticky;
    wire signed [14:0] rdw0, rdw1, rdw2;
    wire signed [3:0]  qy0, qy1, qy2;
    wire [11:0] out_beat;
    wire        out_valid;

    sequencer seq (
        .clk(clk), .rst(rst),
        .csr_a_signed(a_signed), .csr_w_signed(w_signed),
        .csr_row_pe_en(row_en), .csr_col_pe_en(col_en),
        .csr_shift_amt(shift_amt), .csr_relu_en(relu_en),
        .csr_readout_mode(readout_mode), .csr_wb_en(wb_en),
        .csr_input_source(input_source),
        .csr_R(R), .csr_Nb(Nb),
        .cmd_load_tile(c_load), .cmd_run_pass(c_run),
        .cmd_read_band(c_read), .cmd_status(c_stat), .cfg_busy(cfg_busy),
        .g_wrow_data(g_wrow_d), .g_wrow_valid(g_wrow_v),
        .g_vec_data(g_vec_d), .g_vec_valid(g_vec_v),
        .m_wrow_data(m_wrow_d), .m_wrow_valid(m_wrow_v),
        .m_vec_data(m_vec_d), .m_vec_valid(m_vec_v),
        .band_start(band_start), .fetch_tile(fetch_tile), .fetch_vecs(fetch_vecs),
        .mf_busy(mf_busy), .tile_done(tile_done), .vecs_done(vecs_done),
        .a_west0(aw0), .a_west1(aw1), .a_west2(aw2),
        .valid_west(valid_w), .plane_west(plane_w),
        .w_north0(wn0), .w_north1(wn1), .w_north2(wn2),
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel), .stall(stall),
        .bk_wr_en(bk_wr_en), .bk_first_pass(bk_fp), .bk_wr_ptr_rst(bk_wpr),
        .bk_rd_en(seq_rd_en), .bk_rd_addr(seq_rd_addr),
        .bk_rd_valid(bk_rd_valid), .bk_ovf_sticky(ovf_sticky),
        .rd_w0(rdw0), .rd_w1(rdw1), .rd_w2(rdw2),
        .q_y0(qy0), .q_y1(qy1), .q_y2(qy2),
        .wb_start(wb_start), .wb_done(wb_done),
        .out_beat(out_beat), .out_valid(out_valid)
    );

    wb_engine wb (
        .clk(clk), .rst(rst),
        .start(wb_start), .mode_raw(readout_mode), .R4(R[3:0]),
        .base_C(base_C), .done(wb_done), .active(wb_active),
        .wb_rd_en(wb_rd_en), .wb_rd_addr(wb_rd_addr),
        .bk_rd_valid(bk_rd_valid),
        .rd_w0(rdw0), .rd_w1(rdw1), .rd_w2(rdw2),
        .q_y0(qy0), .q_y1(qy1), .q_y2(qy2),
        .wb_req(wb_req), .wb_cmd_write(wb_cmd_write),
        .wb_addr(wb_addr), .wb_len(wb_len), .wb_tx_data(wb_tx),
        .tx_next(tx_next), .sm_busy(sm_busy)
    );

    // ---------------- array ----------------
    wire signed [12:0] fs0, fs1, fs2;
    wire rv0, rv1, rv2;
    systolic_3x3_4bit arr (
        .clk(clk), .rst(rst), .stall(stall),
        .a_signed(a_signed), .w_signed(w_signed),
        .row_pe_en(row_en), .col_pe_en(col_en),
        .w_north0(wn0), .w_north1(wn1), .w_north2(wn2),
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel),
        .a_west0(aw0), .a_west1(aw1), .a_west2(aw2),
        .valid_west0(valid_w), .valid_west1(valid_w), .valid_west2(valid_w),
        .plane_west0(plane_w), .plane_west1(plane_w), .plane_west2(plane_w),
        .sum_north0(13'sd0), .sum_north1(13'sd0), .sum_north2(13'sd0),
        .final_sum0(fs0), .final_sum1(fs1), .final_sum2(fs2),
        .result_valid0(rv0), .result_valid1(rv1), .result_valid2(rv2)
    );

    // ---------------- bank + requant + relu ----------------
    accum_bank_3col #(.BANK_DEPTH(8)) bank (
        .clk(clk), .rst(rst), .stall(stall),
        .wr_en(bk_wr_en), .first_pass(bk_fp), .wr_ptr_rst(bk_wpr),
        .final_sum0(fs0), .result_valid0(rv0),
        .final_sum1(fs1), .result_valid1(rv1),
        .final_sum2(fs2), .result_valid2(rv2),
        .rd_en(wb_active ? wb_rd_en : seq_rd_en),          // [TT-2]
        .rd_addr(wb_active ? wb_rd_addr : seq_rd_addr),
        .rd_data0(rdw0), .rd_data1(rdw1), .rd_data2(rdw2),
        .rd_valid(bk_rd_valid),
        .ovf_sticky0(ovf_sticky[0]), .ovf_sticky1(ovf_sticky[1]),
        .ovf_sticky2(ovf_sticky[2]),
        .wrapped0(), .wrapped1(), .wrapped2()
    );

    wire signed [3:0] rq0, rq1, rq2;
    requant_lane rqA (.shift_amt(shift_amt), .x(rdw0), .y(rq0));
    requant_lane rqB (.shift_amt(shift_amt), .x(rdw1), .y(rq1));
    requant_lane rqC (.shift_amt(shift_amt), .x(rdw2), .y(rq2));
    relu_3col #(.W(4)) rl (
        .relu_en(relu_en),
        .y0_in(rq0), .y1_in(rq1), .y2_in(rq2),
        .valid_in(1'b0),
        .y0_out(qy0), .y1_out(qy1), .y2_out(qy2),
        .valid_out()
    );

    // ---------------- D7 output stage [D7-1] ----------------
    always @(posedge clk) begin
        if (rst) begin
            frame       <= 12'd0;
            frame_valid <= 1'b0;
        end else begin
            frame       <= out_valid ? out_beat : 12'd0;
            frame_valid <= out_valid;
        end
    end

    // lint-quiet unused
    wire _unused_core = &{tile_done, vecs_done, 1'b0};

endmodule

`default_nettype wire