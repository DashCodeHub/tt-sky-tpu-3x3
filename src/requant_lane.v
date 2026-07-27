`default_nettype none
// requant_lane.v  [CD-5, band_engine_spec par.6]
// One requantization lane, extracted from the verified requant_capture_3col
// math (incl. the signed-localparam clamp fix: never compare against
// concatenated constants — concatenations are unsigned and silently make the
// whole comparison unsigned). Pure combinational; instantiate x3 on the BANK
// READOUT path (x = rd_data*, XW=15), relu lanes downstream; raw mode taps
// upstream of this module. shift_amt is static config (CSR SHIFT, id 0x2).
//   y = clamp( (x + 2^(s-1)) >>> s )  to [-2^(YW-1), 2^(YW-1)-1], s=0: no round
module requant_lane #(
    parameter integer XW = 15,
    parameter integer YW = 4
) (
    input  wire [2:0]           shift_amt,
    input  wire signed [XW-1:0] x,
    output wire signed [YW-1:0] y
);
    localparam signed [YW-1:0] YMAX = {1'b0, {(YW-1){1'b1}}};
    localparam signed [YW-1:0] YMIN = {1'b1, {(YW-1){1'b0}}};
    localparam signed [XW:0]   QMAX = YMAX;   // signed extension (the trap fix)
    localparam signed [XW:0]   QMIN = YMIN;

    wire signed [XW:0] half = (shift_amt == 3'd0)
                            ? {(XW+1){1'b0}}
                            : ({{XW{1'b0}}, 1'b1} <<< (shift_amt - 1));
    wire signed [XW:0] r  = {x[XW-1], x} + half;   // XW+1 bits: cannot overflow
    wire signed [XW:0] sh = r >>> shift_amt;

    assign y = (sh > QMAX) ? YMAX
             : (sh < QMIN) ? YMIN
             : sh[YW-1:0];
endmodule
`default_nettype wire