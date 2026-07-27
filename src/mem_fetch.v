`default_nettype none
// mem_fetch.v — Section 10 Step 3 (band_engine_spec v2.1 §7)
// Fetches one band's data from SPI RAM in band order and assembles bytes into
// the SAME internal events the GPIO path produces, so the sequencer stays
// source-agnostic: "weight-row ready" (wrow_valid, 3 per tile, BOTTOM row
// first) and "vector ready" (vec_valid, R per k-slice). One outstanding
// spi_master transaction at a time; SPI slowness surfaces only as event
// spacing (valid bubbles downstream — elasticity is native).
//
// Memory layout (frozen here; host image builder must match):
// [MF-1] Region B: weight tile kb at base_B + 8*kb. 6 payload bytes in the
//        §3.2 packing (byte0={w_r2c1,w_r2c0}, byte1={0,w_r2c2}, bytes2/3 row1,
//        bytes4/5 row0 — BOTTOM FIRST), bytes 6-7 pad (ignored). Stride 8
//        keeps every tile 4-aligned and shift-addressable; padding cost
//        <= 254 B worst case. Only 6 bytes are actually read per tile.
// [MF-2] No multipliers: a_ptr/b_ptr are sequential pointers reloaded from
//        base_A/base_B on band_start and advanced per burst (b_ptr += 8,
//        a_ptr += 2*R). Region A: k-slices consecutive, 2*R bytes each,
//        vector i of slice kb at base_A + 2*(kb*R + i) by construction.
// [MF-3] Pair assembly is shared: event data = {byte1[3:0], byte0[7:0]}
//        ({col2,col1,col0} for weights, {a2,a1,a0} for activations) — the
//        §3.2/§3.3 packings are deliberately identical.
//
// CONTRACT (sequencer side)
// M1. Pulse band_start (>=1 cycle) before the first fetch of a band and after
//     any base_A/base_B rewrite; never while mf_busy.
// M2. fetch_tile / fetch_vecs are single-cycle pulses, accepted only when
//     mf_busy=0; assertions while busy are ignored (sim-warn). Never both in
//     the same cycle.
// M3. R is static during a band (CSR contract), 1..BANK_DEPTH.
// M4. Region placement must satisfy the no-overrun contract (facts §1):
//     base_B + 8*Nb <= 65536 and base_A + 2*R*Nb <= 65536 (sim-checked per
//     burst, not per band).
// M5. tile_done / vecs_done pulse AFTER the SPI transaction fully retires
//     (including cs_gap), so a fetch issued on done honors the recovery gap
//     by construction.

module mem_fetch (
    input  wire        clk,
    input  wire        rst,

    // CSRs (static per band)
    input  wire [15:0] base_A,
    input  wire [15:0] base_B,
    input  wire [5:0]  R,

    // sequencer interface
    input  wire        band_start,
    input  wire        fetch_tile,
    input  wire        fetch_vecs,
    output reg         mf_busy,
    output reg         tile_done,      // pulse [M5]
    output reg         vecs_done,      // pulse [M5]

    // source-agnostic events (into the sequencer's input_source mux)
    output reg  [11:0] wrow_data,
    output reg         wrow_valid,
    output reg  [11:0] vec_data,
    output reg         vec_valid,

    // spi_master command/stream side (cmd_write tied 0 here; writeback engine
    // owns the write path and arbitration lives in the sequencer)
    output reg         sm_req,
    output reg  [15:0] sm_addr,
    output reg  [15:0] sm_len,
    input  wire        sm_busy,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid
);

    localparam [1:0] ST_IDLE = 2'd0,
                     ST_REQ  = 2'd1,   // wait !sm_busy, fire req
                     ST_RECV = 2'd2,   // assemble pairs, count bytes
                     ST_END  = 2'd3;   // wait transaction retire, pulse done

    reg [1:0]  state;
    reg        kind;                   // 0 = tile, 1 = vectors
    reg [15:0] a_ptr, b_ptr;
    reg [6:0]  byte_cnt;               // up to 64 (R=32)
    reg [6:0]  burst_len;
    reg        phase;                  // 0: expect low byte, 1: expect high
    reg [7:0]  pbuf;

    wire [6:0] vlen = {R, 1'b0};       // 2*R bytes

    always @(posedge clk) begin
        if (rst) begin
            state      <= ST_IDLE;
            mf_busy    <= 1'b0;
            tile_done  <= 1'b0;
            vecs_done  <= 1'b0;
            wrow_valid <= 1'b0;
            vec_valid  <= 1'b0;
            sm_req     <= 1'b0;
            sm_addr    <= 16'd0;
            sm_len     <= 16'd0;
            wrow_data  <= 12'd0;
            vec_data   <= 12'd0;
            a_ptr      <= 16'd0;
            b_ptr      <= 16'd0;
            byte_cnt   <= 7'd0;
            burst_len  <= 7'd0;
            phase      <= 1'b0;
            kind       <= 1'b0;
            pbuf       <= 8'd0;
        end else begin
            tile_done  <= 1'b0;
            vecs_done  <= 1'b0;
            wrow_valid <= 1'b0;
            vec_valid  <= 1'b0;
            sm_req     <= 1'b0;

            if (band_start) begin      // [MF-2] pointer reload
                a_ptr <= base_A;
                b_ptr <= base_B;
`ifndef SYNTHESIS
                if (mf_busy) $display("WARN mem_fetch: band_start while busy (M1)");
`endif
            end

            case (state)
            ST_IDLE: begin
                if (fetch_tile) begin
                    kind      <= 1'b0;
                    sm_addr   <= b_ptr;
                    sm_len    <= 16'd6;              // [MF-1] read 6 of 8
                    burst_len <= 7'd6;
                    b_ptr     <= b_ptr + 16'd8;      // [MF-1] stride 8
                    mf_busy   <= 1'b1;
                    byte_cnt  <= 7'd0;
                    phase     <= 1'b0;
                    state     <= ST_REQ;
`ifndef SYNTHESIS
                    if ({1'b0, b_ptr} + 17'd6 > 17'd65536)
                        $display("ERROR mem_fetch: tile burst crosses 0xFFFF (M4)");
`endif
                end else if (fetch_vecs) begin
                    kind      <= 1'b1;
                    sm_addr   <= a_ptr;
                    sm_len    <= {9'd0, vlen};       // 2*R
                    burst_len <= vlen;
                    a_ptr     <= a_ptr + {9'd0, vlen};
                    mf_busy   <= 1'b1;
                    byte_cnt  <= 7'd0;
                    phase     <= 1'b0;
                    state     <= ST_REQ;
`ifndef SYNTHESIS
                    if (R == 6'd0)
                        $display("ERROR mem_fetch: R must be >= 1 (M3)");
                    if ({1'b0, a_ptr} + {10'd0, vlen} > 17'd65536)
                        $display("ERROR mem_fetch: vec burst crosses 0xFFFF (M4)");
`endif
                end
            end

            ST_REQ: begin
                if (!sm_busy) begin
                    sm_req <= 1'b1;                  // one-cycle req (S2)
                    state  <= ST_RECV;
                end
            end

            ST_RECV: begin
                if (rx_valid) begin
                    byte_cnt <= byte_cnt + 7'd1;
                    if (!phase) begin
                        pbuf  <= rx_data;            // low byte
                        phase <= 1'b1;
                    end else begin                   // high byte: emit [MF-3]
                        phase <= 1'b0;
                        if (!kind) begin
                            wrow_data  <= {rx_data[3:0], pbuf};
                            wrow_valid <= 1'b1;
                        end else begin
                            vec_data   <= {rx_data[3:0], pbuf};
                            vec_valid  <= 1'b1;
                        end
                    end
                    if (byte_cnt + 7'd1 == burst_len)
                        state <= ST_END;
                end
            end

            ST_END: begin
                if (!sm_busy) begin                  // [M5] incl. cs_gap
                    if (!kind) tile_done <= 1'b1;
                    else       vecs_done <= 1'b1;
                    mf_busy <= 1'b0;
                    state   <= ST_IDLE;
                end
            end

            default: state <= ST_IDLE;
            endcase

`ifndef SYNTHESIS
            if (state != ST_IDLE && (fetch_tile || fetch_vecs) && !band_start)
                $display("WARN mem_fetch: fetch pulse while busy ignored (M2)");
`endif
        end
    end

endmodule

`default_nettype wire