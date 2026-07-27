`default_nettype none
// pe_44_sIuI_pipelined.v
// Changed to 4x4 multiply
// Pipelined PE with dual-mode (signed/unsigned) multiply, double buffered weights
// Plane select travels with activation: plane_in/plane_out
// row and col signal enables pe to be active
// Parameterized accumulator width, optional saturation, zero-gated MAC, valid-tag

// Changes vs pe_88_sIuI_pipelined_v4
// ------------------------------------------------------------------------------
// [C1*] System wide bit changes from 8 to 4
// [C1] ACCW parameter replaces hardwired 4*DW (default 13 = 8 + clog2(32))
// [C2] a_signed / w_signed mode bits -> 5-bit signed multiply serves both modes
// [C3] Optional saturating accumulate (USE_SAT parameter)
// [C4] Zero-Gating: product_reg holds (no toggle) when activation term is zero;
//      a 1-bit prod_is_zero flag forces 0 into the adder instead
// [C5] valid_in / valid_out tag travels with activation (same pattern as plane tag);
//      invalid data contributes 0 to the sum
//
// USAGE CONTRACT (enforced at MMU level):
//  - row/col enables must be thermometer coded from the top-left edge (e.g. 1110),
//      and static for the duration of a run. plane/valid state is lost while disabled
// - a_signed / w_signed are static configuration, change only between runs.
// - Plane X weight regs may be overwritten no earlier than the cycle AFTER the last plane-X activation is
//      presented at a_in.
// - ACCW sizing: signed-only -> 8 + clog2(K); if unsigned mode is used -> 9 + clog2(K)
//      where K = max accumulation depth. Default 13 covers K = 32 signed / K = 16 unsigned


module pe_44_sIuI_pipelined #(
    parameter integer DW = 4, // [C1*] was 8, changed to 4
    parameter integer ACCW = 13, // [C1*] was 21, changed to 13
    parameter integer USE_SAT = 1 // 1: saturate accumulator, 0: wrap
) (
    input wire clk,
    input wire rst,

    // systolic neighbours
    input wire signed [DW-1:0] a_in,
    input wire signed [DW-1:0] w_in,
    input wire signed [ACCW-1:0] sum_in,
    
    output reg signed [DW-1:0] a_out,
    output reg signed [DW-1:0] w_out,
    output reg signed [ACCW-1:0] sum_out,

    //plane tags travelling with activation
    input wire plane_in, // 0 -> use w_a_reg, 1 -> use w_b_reg
    output reg plane_out,
    input wire valid_in, // [C5] 1 -> real activation, 0 -> bubble (contributes 0)
    output reg valid_out,

    // static datatype configuration (change between runs only)
    input wire a_signed,    // [C2] 1: activations signed, 0: unsigned
    input wire w_signed,    // [C2] 1: weights signed,  0: unsigned 

    // weight double buffer control
    input wire w_load_en,
    input wire w_fill_sel,

    // PE level enable split (reduce fanout): AND locally
    input wire row_pe_en,
    input wire col_pe_en,

    // global pause (level). Functionally safe; at MMU level prefer a root
    // clock gate driven by the same signal 
    input wire stall
);

    // [C2] 5-bit signed x 5-bit signed -> 10-bit product covers all mode combos
    localparam integer PROD_WIDTH = 2*(DW+1);

    // Registers
    reg signed [DW-1:0] a_reg0;
    reg plane_reg0; // Plane tag registered with a_reg0
    reg valid_reg0; // [C5]
    reg signed [DW-1:0] w_a_reg;
    reg signed [DW-1:0] w_b_reg;
    reg signed [ACCW-1:0] sum_in_reg;
    reg signed [ACCW-1:0] sum_in_reg2;
    reg signed [PROD_WIDTH-1:0] product_reg;
    reg prod_is_zero; // [C4] travels in step with product_reg

    // compute-plane weight select uses the registered plane tag
    wire signed [DW-1:0] w_compute = (plane_reg0 == 1'b0) ? w_a_reg : w_b_reg;

    // local enable
    wire pe_en = row_pe_en & col_pe_en;

    // [C2] mode-aware extension to 5 bits (sign-extend if signed, zero-extend if not)
    wire signed [DW:0] a_ext = a_signed ? {a_reg0[DW-1],    a_reg0}    : {1'b0, a_reg0};
    wire signed [DW:0] w_ext = w_signed ? {w_compute[DW-1], w_compute} : {1'b0, w_compute};

    // [C4][C5] the activation term contributes zero if a==0 or data is a bubble
    wire a_term_zero = (a_reg0 == {DW{1'b0}}) | ~valid_reg0;

    // ---------------- Stage 1: input regs, sum pipeline, product, weights ----------------
    always @(posedge clk) begin
        if (rst) begin
            a_reg0       <= {DW{1'b0}};
            plane_reg0   <= 1'b0;
            valid_reg0   <= 1'b0;
            plane_out    <= 1'b0;
            valid_out    <= 1'b0;
 
            a_out        <= {DW{1'b0}};
            w_a_reg      <= {DW{1'b0}};
            w_b_reg      <= {DW{1'b0}};
            w_out        <= {DW{1'b0}};
 
            sum_in_reg   <= {ACCW{1'b0}};
            sum_in_reg2  <= {ACCW{1'b0}};
            product_reg  <= {PROD_WIDTH{1'b0}};
            prod_is_zero <= 1'b1;
        end else if (stall) begin
            // HOLD EVERYTHING (atomic pause; all regs keep their old values)
        end else begin
            // sum pipeline runs whenever not stalled, even when pe_en is low,
            // so partial sums drain through disabled bottom-edge PEs
            sum_in_reg  <= sum_in;
            sum_in_reg2 <= sum_in_reg;
 
            if (pe_en) begin
                // activation/tag forwarding
                a_reg0     <= a_in;
                plane_reg0 <= plane_in;
                valid_reg0 <= valid_in;   // [C5]
 
                a_out      <= a_reg0;
                plane_out  <= plane_reg0;
                valid_out  <= valid_reg0; // [C5]
 
                // [C4] MAC with zero-gating: when the term is zero, HOLD product_reg
                // (no multiplier-result toggle into the flop) and raise the flag;
                // the flag muxes 0 into the adder, preserving correctness.
                prod_is_zero <= a_term_zero;
                if (!a_term_zero)
                    product_reg <= a_ext * w_ext; // [C2] dual-mode multiply
 
                // weights: load + forward during load
                if (w_load_en) begin
                    if (!w_fill_sel) w_a_reg <= w_in;
                    else             w_b_reg <= w_in;
                    w_out <= w_in;
                end
            end else begin
                // DISABLED: no forwarding, MAC contributes nothing
                a_reg0       <= {DW{1'b0}};
                plane_reg0   <= 1'b0;
                valid_reg0   <= 1'b0;
 
                a_out        <= {DW{1'b0}};
                plane_out    <= 1'b0;
                valid_out    <= 1'b0;
 
                prod_is_zero <= 1'b1;     // [C4] flag suffices; no need to clear product_reg
 
                // allow internal preload, but don't forward down
                // (safe only because disabled PEs are bottom/right of active region)
                if (w_load_en) begin
                    if (!w_fill_sel) w_a_reg <= w_in;
                    else             w_b_reg <= w_in;
                end
                w_out <= {DW{1'b0}};
            end
        end
    end
 
    // ---------------- Stage 2: accumulate (optionally saturating) ----------------
    // [C4] flag-selected product: exact 0 for bubbles/zero activations
    wire signed [ACCW-1:0] product_wide =
        prod_is_zero ? {ACCW{1'b0}}
                     : {{(ACCW-PROD_WIDTH){product_reg[PROD_WIDTH-1]}}, product_reg};
 
    // [C3] add in ACCW+1 bits; overflow detected when top two bits differ
    wire signed [ACCW:0]   sum_ext = sum_in_reg2 + product_wide;
    wire                   ovf     = sum_ext[ACCW] ^ sum_ext[ACCW-1];
    wire signed [ACCW-1:0] sum_next =
        (USE_SAT != 0 && ovf)
            ? (sum_ext[ACCW] ? {1'b1, {(ACCW-1){1'b0}}}    // most negative
                             : {1'b0, {(ACCW-1){1'b1}}})   // most positive
            : sum_ext[ACCW-1:0];
 
    always @(posedge clk) begin
        if (rst) begin
            sum_out <= {ACCW{1'b0}};
        end else if (!stall) begin
            if (!pe_en) sum_out <= sum_in_reg2; // passthrough: sums drain through disabled PEs
            else        sum_out <= sum_next;
        end
        // else stall: hold
    end
 
    // ---------------- Elaboration-time sanity checks (simulation only) ----------------
    initial begin
        if (ACCW < PROD_WIDTH) begin
            $display("ERROR: ACCW (%0d) must be >= PROD_WIDTH (%0d)", ACCW, PROD_WIDTH);
            $finish;
        end
    end
endmodule

`default_nettype wire