`timescale 1ns/1ps
// [4B] adjust this include path to your tree, e.g. "../../pe/pe_44_sIuI_pipelined_v1/pe_44_sIuI_pipelined.v"
`include "pe_44_sIuI_pipelined.v"

// systolic_3x3_4bit_v1.v
// [4B] Port of systolic_3x3_v3 from the old v3 8-bit PE to the certified 4-bit
//      pe_44_sIuI_pipelined. Structure, naming, and internal-skew architecture
//      are unchanged. All modifications are tagged [4B]:
//   [4B-1] DW=4 default; ACCW is a parameter (13), no longer 4*DW
//   [4B-2] PE swap: pe_44_sIuI_pipelined, with valid/a_signed/w_signed ports wired
//   [4B-3] BUGFIX: p10 plane-tag skew chain was missing its stall hold
//          (a10/a20/p20 all had it) -> under stall, row-1 activations froze while
//          row-1 plane tags advanced = silent wrong-plane computation after resume
//   [4B-4] valid tag: valid_west inputs, skew chains (0/3/6, WITH stall holds),
//          valid wires through every PE
//   [4B-5] result_valid[per column]: PE(2,c).valid_out delayed by ONE flop to
//          realign with final_sum (valid path is 2-cycle/PE, sum path 3-cycle/PE)
//   [4B-6] a_signed / w_signed static datatype config, broadcast to all PEs
//
// INTERFACE CONTRACTS (unchanged/new):
//   - West inputs are UNSKEWED; this module applies the 3-cycle/row skew internally.
//   - Outputs are SKEWED: column c's results trail column c-1 by 2 cycles
//     (horizontal PE latency). result_valid marks which final_sum cycles are real.
//   - sum_north* must be held 0 (injection timing for nonzero sums not derived).
//   - result_valid relies on row 2 being enabled; sub-array configs
//     (row_pe_en[2]=0) must count result timing controller-side instead.
//   - a_signed/w_signed and row/col enables: static during a run (PE contract).

module systolic_3x3_4bit #(
    parameter DW = 4,        // [4B-1] was 8
    parameter ACCW = 13      // [4B-1] was localparam 4*DW
) (
    input wire clk,
    input wire rst,
    
    input wire [2:0] row_pe_en,
    input wire [2:0] col_pe_en,
    input wire stall,
    
    input wire w_load_en,
    input wire w_fill_sel,

    // [4B-6] static datatype configuration (change between runs only)
    input wire a_signed,
    input wire w_signed,

    input wire signed [DW-1:0] a_west0,
    input wire signed [DW-1:0] a_west1,
    input wire signed [DW-1:0] a_west2,

    // weight plane control to go with activation
    input wire plane_west0,
    input wire plane_west1,
    input wire plane_west2,

    // [4B-4] valid tag travels with activation (1 = real data, 0 = bubble)
    input wire valid_west0,
    input wire valid_west1,
    input wire valid_west2,

    input  wire signed [DW-1:0] w_north0,
    input  wire signed [DW-1:0] w_north1,
    input  wire signed [DW-1:0] w_north2,

    input wire signed [ACCW-1:0] sum_north0,   // must be 0 (see contract)
    input wire signed [ACCW-1:0] sum_north1,
    input wire signed [ACCW-1:0] sum_north2,

    output wire signed [DW-1:0] a_east0,
    output wire signed [DW-1:0] a_east1,
    output wire signed [DW-1:0] a_east2,

    output wire signed [DW-1:0] w_south0,
    output wire signed [DW-1:0] w_south1,
    output wire signed [DW-1:0] w_south2,

    output wire signed [ACCW-1:0] final_sum0,
    output wire signed [ACCW-1:0] final_sum1,
    output wire signed [ACCW-1:0] final_sum2,

    // [4B-5] high exactly on the cycles when the matching final_sum is a real result
    output reg result_valid0,
    output reg result_valid1,
    output reg result_valid2
);

    // Activation wires (three rows, four taps each)
    wire signed [DW-1:0] a00, a01, a02, a03;
    wire signed [DW-1:0] a10, a11, a12, a13;
    wire signed [DW-1:0] a20, a21, a22, a23;

    // assign a00 = a_west0
    assign a_east0 = a03;
    assign a_east1 = a13;
    assign a_east2 = a23;

    // Weight wires (four rows, three taps each)
    wire signed [DW-1:0] w00, w01, w02;
    wire signed [DW-1:0] w10, w11, w12;
    wire signed [DW-1:0] w20, w21, w22;
    wire signed [DW-1:0] w30, w31, w32;

    assign w00 = w_north0;
    assign w01 = w_north1;
    assign w02 = w_north2;
    assign w_south0 = w30;
    assign w_south1 = w31;
    assign w_south2 = w32;


    // Partial-sum wires 
    wire signed [ACCW-1:0] s00, s01, s02;
    wire signed [ACCW-1:0] s10, s11, s12;
    wire signed [ACCW-1:0] s20, s21, s22;
    wire signed [ACCW-1:0] s30, s31, s32;

    assign s00 = sum_north0;
    assign s01 = sum_north1;
    assign s02 = sum_north2;

    // weight plane wires
    wire p00, p01, p02, p03;
    wire p10, p11, p12, p13;
    wire p20, p21, p22, p23;

    // [4B-4] valid wires (parallel to plane wires)
    wire v00, v01, v02, v03;
    wire v10, v11, v12, v13;
    wire v20, v21, v22, v23;


    // Row 0 PEs
    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe00 (   // [4B-2]
        .clk(clk), .rst(rst), 
        .row_pe_en(row_pe_en[0]),
        .col_pe_en(col_pe_en[0]),
        .stall(stall),
        .a_in(a00), .w_in(w00), .sum_in(s00),
        .a_out(a01), .w_out(w10), .sum_out(s10),
        .plane_in(p00), .plane_out(p01),
        .valid_in(v00), .valid_out(v01),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe01 (   // [4B-2]
        .clk(clk), .rst(rst), 
        .row_pe_en(row_pe_en[0]),
        .col_pe_en(col_pe_en[1]),
        .stall(stall),
        .a_in(a01), .w_in(w01), .sum_in(s01),
        .a_out(a02), .w_out(w11), .sum_out(s11),
        .plane_in(p01), .plane_out(p02),
        .valid_in(v01), .valid_out(v02),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe02 (   // [4B-2]
        .clk(clk), .rst(rst),
        .row_pe_en(row_pe_en[0]),
        .col_pe_en(col_pe_en[2]),
        .stall(stall),
        .a_in(a02), .w_in(w02), .sum_in(s02),
        .a_out(a03), .w_out(w12), .sum_out(s12),
        .plane_in(p02), .plane_out(p03),
        .valid_in(v02), .valid_out(v03),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    // Row 1 PEs
    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe10 (   // [4B-2]
        .clk(clk), .rst(rst),
        .row_pe_en(row_pe_en[1]),
        .col_pe_en(col_pe_en[0]),
        .stall(stall),
        .a_in(a10), .w_in(w10), .sum_in(s10),
        .a_out(a11), .w_out(w20), .sum_out(s20),
        .plane_in(p10), .plane_out(p11),
        .valid_in(v10), .valid_out(v11),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe11 (   // [4B-2]
        .clk(clk), .rst(rst),
        .row_pe_en(row_pe_en[1]),
        .col_pe_en(col_pe_en[1]),
        .stall(stall),
        .a_in(a11), .w_in(w11), .sum_in(s11),
        .a_out(a12), .w_out(w21), .sum_out(s21),
        .plane_in(p11), .plane_out(p12),
        .valid_in(v11), .valid_out(v12),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe12 (   // [4B-2]
        .clk(clk), .rst(rst),
        .row_pe_en(row_pe_en[1]),
        .col_pe_en(col_pe_en[2]),
        .stall(stall),
        .a_in(a12), .w_in(w12), .sum_in(s12),
        .a_out(a13), .w_out(w22), .sum_out(s22),
        .plane_in(p12), .plane_out(p13),
        .valid_in(v12), .valid_out(v13),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    // Row 2 PEs
    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe20 (   // [4B-2]
        .clk(clk), .rst(rst),
        .row_pe_en(row_pe_en[2]),
        .col_pe_en(col_pe_en[0]),
        .stall(stall),
        .a_in(a20), .w_in(w20), .sum_in(s20),
        .a_out(a21), .w_out(w30), .sum_out(s30),
        .plane_in(p20), .plane_out(p21),
        .valid_in(v20), .valid_out(v21),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe21 (   // [4B-2]
        .clk(clk), .rst(rst),
        .row_pe_en(row_pe_en[2]),
        .col_pe_en(col_pe_en[1]),
        .stall(stall),
        .a_in(a21), .w_in(w21), .sum_in(s21),
        .a_out(a22), .w_out(w31), .sum_out(s31),
        .plane_in(p21), .plane_out(p22),
        .valid_in(v21), .valid_out(v22),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    pe_44_sIuI_pipelined #(.DW(DW), .ACCW(ACCW)) pe22 (   // [4B-2]
        .clk(clk), .rst(rst),
        .row_pe_en(row_pe_en[2]),
        .col_pe_en(col_pe_en[2]),
        .stall(stall),
        .a_in(a22), .w_in(w22), .sum_in(s22),
        .a_out(a23), .w_out(w32), .sum_out(s32),
        .plane_in(p22), .plane_out(p23),
        .valid_in(v22), .valid_out(v23),                  // [4B-4]
        .a_signed(a_signed), .w_signed(w_signed),         // [4B-6]
        .w_load_en(w_load_en), .w_fill_sel(w_fill_sel)
    );

    assign final_sum0 = s30;
    assign final_sum1 = s31;
    assign final_sum2 = s32;

    // -------------------------
    // Activation Input delays
    // -------------------------
    // Row 0 -> no delay
    assign a00 = a_west0;

    // Row 1 -> 3-clock delay
    
    // new piece of skew
    reg signed [DW-1:0] a10_d1, a10_d2, a10_d3;
    always @(posedge clk) begin 
        if (rst) begin 
            a10_d1 <= 0;
            a10_d2 <= 0;
            a10_d3 <= 0;
        end else if (stall) begin
            // hold
        end else begin 
            a10_d1 <= a_west1;
            a10_d2 <= a10_d1;
            a10_d3 <= a10_d2;
        end
    end
    assign a10 = a10_d3;

    // Row 2 -> 6-clock delay
    reg signed [DW-1:0] a20_d1, a20_d2, a20_d3, a20_d4, a20_d5, a20_d6;
    always @(posedge clk) begin 
        if (rst) begin 
            a20_d1 <= 0; a20_d2 <= 0; a20_d3 <= 0; a20_d4 <= 0; a20_d5 <= 0; a20_d6 <= 0;
        end else if (stall) begin
            // hold
        end else begin 
            a20_d1 <= a_west2;
            a20_d2 <= a20_d1;
            a20_d3 <= a20_d2;
            a20_d4 <= a20_d3;
            a20_d5 <= a20_d4;
            a20_d6 <= a20_d5;
        end
    end
    assign a20 = a20_d6;

    // -------------------------------------
    // activation plane delays (3/6 skew)
    // -------------------------------------
    assign p00 = plane_west0;

    // Row1 plane tag 3-cycle
    reg p10_d1, p10_d2, p10_d3;
    always @(posedge clk) begin
        if (rst) begin
            p10_d1 <= 1'b0;
            p10_d2 <= 1'b0;
            p10_d3 <= 1'b0;
        end else if (stall) begin
            // hold                              // [4B-3] BUGFIX: was missing
        end else begin
            p10_d1 <= plane_west1;
            p10_d2 <= p10_d1;
            p10_d3 <= p10_d2;
        end
    end
    assign p10 = p10_d3;

    // Row2 plane tag 6-cycle
    reg p20_d1, p20_d2, p20_d3, p20_d4, p20_d5, p20_d6;
    always @(posedge clk) begin
        if (rst) begin
            p20_d1 <= 1'b0; p20_d2 <= 1'b0; p20_d3 <= 1'b0;
            p20_d4 <= 1'b0; p20_d5 <= 1'b0; p20_d6 <= 1'b0;
        end else if (stall) begin
            // hold
        end else begin
            p20_d1 <= plane_west2;
            p20_d2 <= p20_d1;
            p20_d3 <= p20_d2;
            p20_d4 <= p20_d3;
            p20_d5 <= p20_d4;
            p20_d6 <= p20_d5;
        end
    end
    assign p20 = p20_d6;

    // -------------------------------------
    // [4B-4] valid delays (0/3/6 skew, same pattern as plane tags)
    // -------------------------------------
    assign v00 = valid_west0;

    // Row1 valid tag 3-cycle
    reg v10_d1, v10_d2, v10_d3;
    always @(posedge clk) begin
        if (rst) begin
            v10_d1 <= 1'b0;
            v10_d2 <= 1'b0;
            v10_d3 <= 1'b0;
        end else if (stall) begin
            // hold
        end else begin
            v10_d1 <= valid_west1;
            v10_d2 <= v10_d1;
            v10_d3 <= v10_d2;
        end
    end
    assign v10 = v10_d3;

    // Row2 valid tag 6-cycle
    reg v20_d1, v20_d2, v20_d3, v20_d4, v20_d5, v20_d6;
    always @(posedge clk) begin
        if (rst) begin
            v20_d1 <= 1'b0; v20_d2 <= 1'b0; v20_d3 <= 1'b0;
            v20_d4 <= 1'b0; v20_d5 <= 1'b0; v20_d6 <= 1'b0;
        end else if (stall) begin
            // hold
        end else begin
            v20_d1 <= valid_west2;
            v20_d2 <= v20_d1;
            v20_d3 <= v20_d2;
            v20_d4 <= v20_d3;
            v20_d5 <= v20_d4;
            v20_d6 <= v20_d5;
        end
    end
    assign v20 = v20_d6;

    // -------------------------------------------------------------------
    // [4B-5] result_valid: row-2 valid_out realigned to the sum path.
    // valid_out of PE(2,c) is a 2-cycle path; its sum contribution exits
    // sum_out one cycle later (3-cycle path) -> delay valid by one flop.
    // -------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            result_valid0 <= 1'b0;
            result_valid1 <= 1'b0;
            result_valid2 <= 1'b0;
        end else if (stall) begin
            // hold
        end else begin
            result_valid0 <= v21;
            result_valid1 <= v22;
            result_valid2 <= v23;
        end
    end

endmodule