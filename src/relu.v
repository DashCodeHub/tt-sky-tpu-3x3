`default_nettype none
// relu.v
// Optional ReLU for the requant/readout path (roadmap item 3: "ReLU mux
// (1 gate) while there"). Two modules:
//
//   relu       : generic single-lane, parameterized width. Pure combinational,
//                zero flops. out = (relu_en && in < 0) ? 0 : in, i.e. one
//                sign-gated AND-mask per bit.
//   relu_3col  : drop-in 3-column wrapper matching the y0..y2/out_valid shape
//                of requant_capture_3col (valid passes through untouched --
//                ReLU changes values, never timing).
//
// PLACEMENT (decision on record): instantiate POST-requant on the 4-bit y
// lanes, not pre-requant on the wide sums. ReLU-then-requant and
// requant-then-ReLU are mathematically identical here (any negative sum
// requantizes to y <= 0; clamping either side at zero yields the same
// result, since requant(0) = 0 for all shift_amt), and the post-requant
// placement is the narrower mux. The generic width still allows a BW=15
// pre-requant instance if a future path wants one.
//
// USAGE CONTRACT
// ------------------------------------------------------------------------------
// R1. relu_en is STATIC CONFIG (same class as a_signed/w_signed/shift_amt):
//     change only between runs; false-path it in the SDC alongside the other
//     config pins.
// R2. Combinational: adds one mux level to whatever path feeds it. Post-
//     requant it lands on the registered y* outputs, i.e. on an output/IO
//     path, never on the array's critical set.
// R3. relu_en=0 is bit-exact passthrough.

module relu #(
    parameter integer W = 4
) (
    input  wire                relu_en,   // static config (R1)
    input  wire signed [W-1:0] in,
    output wire signed [W-1:0] out
);
    // negative iff sign bit set; kill the whole word when enabled & negative
    assign out = (relu_en & in[W-1]) ? {W{1'b0}} : in;
endmodule


module relu_3col #(
    parameter integer W = 4
) (
    input  wire                relu_en,
    input  wire signed [W-1:0] y0_in,
    input  wire signed [W-1:0] y1_in,
    input  wire signed [W-1:0] y2_in,
    input  wire                valid_in,
    output wire signed [W-1:0] y0_out,
    output wire signed [W-1:0] y1_out,
    output wire signed [W-1:0] y2_out,
    output wire                valid_out
);
    relu #(.W(W)) r0 (.relu_en(relu_en), .in(y0_in), .out(y0_out));
    relu #(.W(W)) r1 (.relu_en(relu_en), .in(y1_in), .out(y1_out));
    relu #(.W(W)) r2 (.relu_en(relu_en), .in(y2_in), .out(y2_out));
    assign valid_out = valid_in;   // ReLU never touches timing
endmodule

`default_nettype wire