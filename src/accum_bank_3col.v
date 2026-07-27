`default_nettype none
// accum_bank_3col.v
// Per-column post-array accumulator banks for systolic_3x3_4bit (decision D12,
// TPUv1-style). Absorbs the array's staggered column outputs with per-column
// write pointers, accumulates partial sums across k-tiles in BW-bit signed
// saturating registers, and doubles as the output buffer (requant applies at
// READOUT, downstream of this module).
//
// Design tags
// ------------------------------------------------------------------------------
// [AB1] Geometry: BANK_DEPTH rows x 3 columns x BW bits. BW sizing rule:
//       BW = 8 + clog2(K_total) where K_total is the TOTAL accumulation depth
//       across all tiles (array-internal K=3 lives inside ACCW=13 and never
//       saturates; cross-tile K=128 -> BW=15 default). Same sizing family as
//       the PE ACCW rule.
// [AB2] Per-column write pointers keyed on the column's OWN result_valid.
//       The array's outputs are column-staggered (col c trails c-1 by 2
//       cycles); because each column counts only its own valids, the stagger
//       is absorbed for free and NO de-skew block is needed (D12).
// [AB3] first_pass: 1 -> LOAD (overwrite row with the incoming sum: this is
//       how a new output tile "clears" the bank without a clear pass),
//       0 -> ACCUMULATE (row += incoming sum, saturating per [AB4]).
// [AB4] Saturating accumulate (USE_SAT=1): add in BW+1 bits, overflow when
//       the top two bits differ, clamp to +/- full scale (same idiom as the
//       PE stage-2 accumulator). USE_SAT=0 wraps. ovf_sticky* telemetry flags
//       latch any accumulate-phase overflow until wr_ptr_rst/rst.
// [AB5] stall: ATOMIC HOLD of the entire module, same global-pause domain as
//       the array. This is a correctness requirement, not a convenience:
//       result_valid* are held LEVELS under stall, so a free-running bank
//       would re-accumulate the same held result once per stall cycle.
// [AB6] Bank CONTENTS are NOT reset by rst (deliberate: clearing
//       3*BANK_DEPTH*BW flops would load the rst broadcast tree, the same
//       fanout network items 2.10/D8 manage; first_pass makes a data clear
//       unnecessary). Contents are UNDEFINED after rst until a first_pass
//       pass has written each row that will later be read. Pointers, flags,
//       and the read port ARE reset.
// [AB7] Registered read port: rd_data*/rd_valid are valid one cycle after
//       rd_en (keeps the BANK_DEPTH:1 read mux out of the downstream
//       requant's timing budget). Reads never disturb the write side.
//
// USAGE CONTRACT (enforced at control-block level)
// ------------------------------------------------------------------------------
// C1. first_pass is static for the duration of a pass; change it only between
//     passes (i.e., around wr_ptr_rst boundaries), never mid-stream.
// C2. Exactly BANK_DEPTH valid results per column per pass. The pointer wraps
//     mod BANK_DEPTH (wrapped* flags latch the wrap); results beyond
//     BANK_DEPTH per pass silently overwrite/accumulate onto row 0 onward.
// C3. stall must be the SAME signal (same cycle alignment) the array sees.
// C4. wr_ptr_rst and an accepted write must not be asserted in the same
//     cycle (rst-priority: the write would be dropped). Pulse wr_ptr_rst in
//     the gap between passes.
// C5. Reading a row in the same cycle it is written returns the OLD value
//     (pre-edge state). Defined, but the intended usage separates compute
//     and readout phases per bank region.
// C6. wr_ptr_rst does NOT clear bank contents ([AB6]); data freshness is
//     first_pass's job.
// C7. Do not override the AW parameter; it is derived from BANK_DEPTH.

module accum_bank_3col #(
    parameter integer BANK_DEPTH = 32,                                   // [AB1] rows per column (any >=1, non-pow2 OK)
    parameter integer SUMW       = 13,                                   // array final_sum width (ACCW)
    parameter integer BW         = 15,                                   // [AB1] bank word width = 8 + clog2(K_total)
    parameter integer USE_SAT    = 1,                                    // [AB4] 1: saturate, 0: wrap
    parameter integer AW = (BANK_DEPTH < 2) ? 1 : $clog2(BANK_DEPTH)     // derived; do not override (C7)
) (
    input  wire clk,
    input  wire rst,
    input  wire stall,                       // [AB5] atomic hold, shared with array

    // ---- write side: from array (column-staggered, [AB2]) ----
    input  wire wr_en,                       // pass-level gate from control block
    input  wire first_pass,                  // [AB3] 1: load, 0: accumulate (static per pass, C1)
    input  wire wr_ptr_rst,                  // sync pulse: zero pointers + flags (C4, C6)
    input  wire signed [SUMW-1:0] final_sum0,
    input  wire signed [SUMW-1:0] final_sum1,
    input  wire signed [SUMW-1:0] final_sum2,
    input  wire result_valid0,
    input  wire result_valid1,
    input  wire result_valid2,

    // ---- read side ([AB7]) ----
    input  wire rd_en,
    input  wire [AW-1:0] rd_addr,            // shared row address, all 3 columns read in parallel
    output reg  signed [BW-1:0] rd_data0,
    output reg  signed [BW-1:0] rd_data1,
    output reg  signed [BW-1:0] rd_data2,
    output reg  rd_valid,                    // rd_en delayed 1 cycle

    // ---- status: control-block visibility ----
    output reg  [AW-1:0] wr_ptr0,
    output reg  [AW-1:0] wr_ptr1,
    output reg  [AW-1:0] wr_ptr2,
    output reg  wrapped0,                    // sticky: pointer wrapped since last wr_ptr_rst (pass-complete marker, C2)
    output reg  wrapped1,
    output reg  wrapped2,
    output reg  ovf_sticky0,                 // [AB4] sticky: accumulate-phase overflow occurred since last wr_ptr_rst
    output reg  ovf_sticky1,
    output reg  ovf_sticky2
);

    // saturation rails [AB4]
    localparam signed [BW-1:0] SAT_MAX = {1'b0, {(BW-1){1'b1}}};
    localparam signed [BW-1:0] SAT_MIN = {1'b1, {(BW-1){1'b0}}};

    // storage: 3 x BANK_DEPTH x BW flop arrays ([AB6]: never reset)
    reg signed [BW-1:0] bank0 [0:BANK_DEPTH-1];
    reg signed [BW-1:0] bank1 [0:BANK_DEPTH-1];
    reg signed [BW-1:0] bank2 [0:BANK_DEPTH-1];

    // ------------------------------------------------------------------
    // write-side combinational, per column (unrolled, house style)
    // read-modify-write path: wr_ptr -> BANK_DEPTH:1 mux -> BW+1-bit add
    // -> saturate mux -> write demux. This is the module's timing path.
    // ------------------------------------------------------------------
    // column 0
    wire signed [BW-1:0] fs0_ext  = {{(BW-SUMW){final_sum0[SUMW-1]}}, final_sum0};
    wire signed [BW-1:0] cur0     = bank0[wr_ptr0];
    wire signed [BW:0]   acc0     = cur0 + fs0_ext;
    wire                 ovf0     = acc0[BW] ^ acc0[BW-1];
    wire signed [BW-1:0] accsat0  = (USE_SAT != 0 && ovf0)
                                    ? (acc0[BW] ? SAT_MIN : SAT_MAX)
                                    : acc0[BW-1:0];
    wire signed [BW-1:0] wdata0   = first_pass ? fs0_ext : accsat0;   // [AB3]
    wire                 wfire0   = wr_en & result_valid0;            // [AB2]

    // column 1
    wire signed [BW-1:0] fs1_ext  = {{(BW-SUMW){final_sum1[SUMW-1]}}, final_sum1};
    wire signed [BW-1:0] cur1     = bank1[wr_ptr1];
    wire signed [BW:0]   acc1     = cur1 + fs1_ext;
    wire                 ovf1     = acc1[BW] ^ acc1[BW-1];
    wire signed [BW-1:0] accsat1  = (USE_SAT != 0 && ovf1)
                                    ? (acc1[BW] ? SAT_MIN : SAT_MAX)
                                    : acc1[BW-1:0];
    wire signed [BW-1:0] wdata1   = first_pass ? fs1_ext : accsat1;
    wire                 wfire1   = wr_en & result_valid1;

    // column 2
    wire signed [BW-1:0] fs2_ext  = {{(BW-SUMW){final_sum2[SUMW-1]}}, final_sum2};
    wire signed [BW-1:0] cur2     = bank2[wr_ptr2];
    wire signed [BW:0]   acc2     = cur2 + fs2_ext;
    wire                 ovf2     = acc2[BW] ^ acc2[BW-1];
    wire signed [BW-1:0] accsat2  = (USE_SAT != 0 && ovf2)
                                    ? (acc2[BW] ? SAT_MIN : SAT_MAX)
                                    : acc2[BW-1:0];
    wire signed [BW-1:0] wdata2   = first_pass ? fs2_ext : accsat2;
    wire                 wfire2   = wr_en & result_valid2;

    // ------------------------------------------------------------------
    // write side sequential
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            wr_ptr0 <= {AW{1'b0}}; wrapped0 <= 1'b0; ovf_sticky0 <= 1'b0;
            wr_ptr1 <= {AW{1'b0}}; wrapped1 <= 1'b0; ovf_sticky1 <= 1'b0;
            wr_ptr2 <= {AW{1'b0}}; wrapped2 <= 1'b0; ovf_sticky2 <= 1'b0;
            // [AB6] bank contents deliberately NOT reset
        end else if (stall) begin
            // [AB5] HOLD EVERYTHING (atomic pause; prevents re-accumulating
            // the held result_valid level)
        end else begin
            if (wr_ptr_rst) begin                       // C4: priority over writes
                wr_ptr0 <= {AW{1'b0}}; wrapped0 <= 1'b0; ovf_sticky0 <= 1'b0;
                wr_ptr1 <= {AW{1'b0}}; wrapped1 <= 1'b0; ovf_sticky1 <= 1'b0;
                wr_ptr2 <= {AW{1'b0}}; wrapped2 <= 1'b0; ovf_sticky2 <= 1'b0;
            end else begin
                if (wfire0) begin
                    bank0[wr_ptr0] <= wdata0;
                    if (wr_ptr0 == BANK_DEPTH-1) begin
                        wr_ptr0 <= {AW{1'b0}}; wrapped0 <= 1'b1;    // C2
                    end else begin
                        wr_ptr0 <= wr_ptr0 + 1'b1;
                    end
                    if (!first_pass && ovf0) ovf_sticky0 <= 1'b1;   // [AB4] telemetry
                end
                if (wfire1) begin
                    bank1[wr_ptr1] <= wdata1;
                    if (wr_ptr1 == BANK_DEPTH-1) begin
                        wr_ptr1 <= {AW{1'b0}}; wrapped1 <= 1'b1;
                    end else begin
                        wr_ptr1 <= wr_ptr1 + 1'b1;
                    end
                    if (!first_pass && ovf1) ovf_sticky1 <= 1'b1;
                end
                if (wfire2) begin
                    bank2[wr_ptr2] <= wdata2;
                    if (wr_ptr2 == BANK_DEPTH-1) begin
                        wr_ptr2 <= {AW{1'b0}}; wrapped2 <= 1'b1;
                    end else begin
                        wr_ptr2 <= wr_ptr2 + 1'b1;
                    end
                    if (!first_pass && ovf2) ovf_sticky2 <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // read side ([AB7]): registered, 1-cycle latency, old-data on same-cycle
    // RAW (C5). rd_data* intentionally not reset (X-until-first-read is
    // gated by rd_valid).
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            rd_valid <= 1'b0;
        end else if (!stall) begin
            rd_valid <= rd_en;
            if (rd_en) begin
                rd_data0 <= bank0[rd_addr];
                rd_data1 <= bank1[rd_addr];
                rd_data2 <= bank2[rd_addr];
            end
        end
        // else stall: hold [AB5]
    end

    // ------------------------------------------------------------------
    // elaboration-time sanity checks (simulation only; SYNTHESIS-guarded per
    // the 4-bit PE punch-list lesson: this file will be `include'd)
    // ------------------------------------------------------------------
`ifndef SYNTHESIS
    initial begin
        if (BW < SUMW) begin
            $display("ERROR: BW (%0d) must be >= SUMW (%0d)", BW, SUMW);
            $finish;
        end
        if (BANK_DEPTH < 1) begin
            $display("ERROR: BANK_DEPTH (%0d) must be >= 1", BANK_DEPTH);
            $finish;
        end
        if (AW < ((BANK_DEPTH < 2) ? 1 : $clog2(BANK_DEPTH))) begin
            $display("ERROR: AW (%0d) too small for BANK_DEPTH (%0d) - do not override AW", AW, BANK_DEPTH);
            $finish;
        end
    end
`endif

endmodule

`default_nettype wire