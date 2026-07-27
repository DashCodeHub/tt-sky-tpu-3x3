`default_nettype none
// cfg_decoder.v — input-beat parser + CSR file (band_engine_spec v2.1 §3,
// csr_control_spec §2 id map, BANK_DEPTH=8). Samples one 8-bit beat per
// cycle (host owns the clock; idle bus = NOP 0x00). Produces: CSR values,
// command pulses, cfg_busy, and the GPIO-source events (weight rows and
// activation vectors) using the SAME pairing as mem_fetch ([MF-3]).
//
// [CFG-1] CSR writes take effect on the value byte's cycle; host contract:
//         CFG only while the sequencer is idle (config is static-per-run/
//         band). Unknown CSR id: value bytes consumed and dropped (sim-warn).
// [CFG-2] LOAD_TILE payload = 6 bytes -> 3 wrow events (bottom row first);
//         RUN_PASS payload = 2*R bytes -> R vec events (GPIO mode only; in
//         SPI mode RUN_PASS is header-only per §3.1 and the pulse fires
//         immediately).
// [CFG-3] Thermometer-shape sim-assert on ENABLES writes (tracker item 2.3).

module cfg_decoder (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  in_beat,

    // CSR outputs (resets per csr_control_spec §2)
    output reg         csr_a_signed,
    output reg         csr_w_signed,
    output reg         csr_relu_en,
    output reg         csr_readout_mode,
    output reg         csr_input_source,
    output reg         csr_spi_fast_read,
    output reg         csr_wb_en,
    output reg  [2:0]  csr_row_pe_en,
    output reg  [2:0]  csr_col_pe_en,
    output reg  [2:0]  csr_shift_amt,
    output reg  [5:0]  csr_R,
    output reg  [6:0]  csr_Nb,
    output reg  [7:0]  csr_spi_div,
    output reg  [15:0] csr_spi_cs_gap,
    output reg  [15:0] csr_base_A,
    output reg  [15:0] csr_base_B,
    output reg  [15:0] csr_base_C,

    // command pulses + busy
    output reg         cmd_load_tile,
    output reg         cmd_run_pass,
    output reg         cmd_read_band,
    output reg         cmd_status,
    output reg         cfg_busy,

    // GPIO-source events
    output reg  [11:0] g_wrow_data,
    output reg         g_wrow_valid,
    output reg  [11:0] g_vec_data,
    output reg         g_vec_valid
);
    localparam [1:0] D_IDLE = 2'd0, D_CFG = 2'd1, D_WPAY = 2'd2, D_APAY = 2'd3;

    reg [1:0] dstate;
    reg [3:0] cfg_id;
    reg       cfg_two, cfg_second;    // 2-byte CSR, expecting 2nd byte
    reg [7:0] lowbyte;                // 1st byte of 16-bit CSR / of a pair
    reg [4:0] pay_cnt;                // up to 16 payload bytes (2*R, R<=8)
    reg       phase;                  // pair phase (shared with [MF-3])

    wire [4:0] apay_len = {csr_R[3:0], 1'b0};   // 2*R

    always @(posedge clk) begin
        if (rst) begin
            dstate <= D_IDLE; cfg_busy <= 0;
            cmd_load_tile <= 0; cmd_run_pass <= 0;
            cmd_read_band <= 0; cmd_status <= 0;
            g_wrow_valid <= 0; g_vec_valid <= 0;
            g_wrow_data <= 0; g_vec_data <= 0;
            cfg_id <= 0; cfg_two <= 0; cfg_second <= 0;
            lowbyte <= 0; pay_cnt <= 0; phase <= 0;
            // CSR resets (csr_control_spec §2)
            csr_a_signed <= 0; csr_w_signed <= 0; csr_relu_en <= 0;
            csr_readout_mode <= 0; csr_input_source <= 0;
            csr_spi_fast_read <= 1; csr_wb_en <= 0;
            csr_row_pe_en <= 3'b111; csr_col_pe_en <= 3'b111;
            csr_shift_amt <= 0; csr_R <= 6'd1; csr_Nb <= 7'd1;
            csr_spi_div <= 8'd3; csr_spi_cs_gap <= 16'd255;
            csr_base_A <= 16'h0000; csr_base_B <= 16'h6000;
            csr_base_C <= 16'hC000;
        end else begin
            cmd_load_tile <= 0; cmd_run_pass <= 0;
            cmd_read_band <= 0; cmd_status <= 0;
            g_wrow_valid <= 0; g_vec_valid <= 0;

            case (dstate)
            D_IDLE: begin
                cfg_busy <= 0;
                case (in_beat[7:4])
                4'h0: ;                                   // NOP
                4'h1: begin                               // CFG
                    cfg_id  <= in_beat[3:0];
                    cfg_two <= (in_beat[3:0] >= 4'h6 && in_beat[3:0] <= 4'h9);
                    cfg_second <= 1'b0;
                    cfg_busy   <= 1'b1;
                    dstate     <= D_CFG;
                end
                4'h2: begin                               // LOAD_TILE
                    cmd_load_tile <= 1'b1;
                    if (!csr_input_source) begin          // [CFG-2]
                        pay_cnt <= 5'd0; phase <= 1'b0;
                        dstate  <= D_WPAY;
                    end
                end
                4'h3: begin                               // RUN_PASS
                    cmd_run_pass <= 1'b1;
                    if (!csr_input_source) begin
                        pay_cnt <= 5'd0; phase <= 1'b0;
                        dstate  <= D_APAY;
                    end
                end
                4'h4: cmd_read_band <= 1'b1;
                4'h5: cmd_status    <= 1'b1;
                default: begin
`ifndef SYNTHESIS
                    $display("WARN cfg_decoder: unknown opcode %h", in_beat[7:4]);
`endif
                end
                endcase
            end

            D_CFG: begin
                if (cfg_two && !cfg_second) begin
                    lowbyte <= in_beat; cfg_second <= 1'b1;   // LE low byte
                end else begin
                    case (cfg_id)
                    4'h0: begin
                        csr_a_signed      <= in_beat[0];
                        csr_w_signed      <= in_beat[1];
                        csr_relu_en       <= in_beat[2];
                        csr_readout_mode  <= in_beat[3];
                        csr_input_source  <= in_beat[4];
                        csr_spi_fast_read <= in_beat[5];
                        csr_wb_en         <= in_beat[6];
                    end
                    4'h1: begin
                        csr_row_pe_en <= in_beat[2:0];
                        csr_col_pe_en <= in_beat[5:3];
`ifndef SYNTHESIS
                        if (in_beat[2:0]==3'b101 || in_beat[2:0]==3'b010 ||
                            in_beat[5:3]==3'b101 || in_beat[5:3]==3'b010)
                            $display("ERROR cfg_decoder: non-thermometer mask (CFG-3)");
`endif
                    end
                    4'h2: csr_shift_amt <= in_beat[2:0];
                    4'h3: begin
                        csr_R <= in_beat[5:0];
`ifndef SYNTHESIS
                        if (in_beat[5:0]==6'd0 || in_beat[5:0]>6'd8)
                            $display("ERROR cfg_decoder: R=%0d out of 1..8", in_beat[5:0]);
`endif
                    end
                    4'h4: csr_Nb        <= in_beat[6:0];
                    4'h5: csr_spi_div   <= in_beat;
                    4'h6: csr_spi_cs_gap<= {in_beat, lowbyte};
                    4'h7: csr_base_A    <= {in_beat, lowbyte};
                    4'h8: csr_base_B    <= {in_beat, lowbyte};
                    4'h9: csr_base_C    <= {in_beat, lowbyte};
                    default: begin
`ifndef SYNTHESIS
                        $display("WARN cfg_decoder: reserved CSR id %h dropped", cfg_id);
`endif
                    end
                    endcase
                    cfg_busy <= 1'b0;
                    dstate   <= D_IDLE;
                end
            end

            D_WPAY: begin                                  // 6 bytes -> 3 rows
                pay_cnt <= pay_cnt + 5'd1;
                if (!phase) begin
                    lowbyte <= in_beat; phase <= 1'b1;
                end else begin
                    phase <= 1'b0;
                    g_wrow_data  <= {in_beat[3:0], lowbyte};   // [MF-3]
                    g_wrow_valid <= 1'b1;
                end
                if (pay_cnt + 5'd1 == 5'd6) dstate <= D_IDLE;
            end

            D_APAY: begin                                  // 2*R bytes -> R vecs
                pay_cnt <= pay_cnt + 5'd1;
                if (!phase) begin
                    lowbyte <= in_beat; phase <= 1'b1;
                end else begin
                    phase <= 1'b0;
                    g_vec_data  <= {in_beat[3:0], lowbyte};    // [MF-3]
                    g_vec_valid <= 1'b1;
                end
                if (pay_cnt + 5'd1 == apay_len) dstate <= D_IDLE;
            end
            endcase
        end
    end

endmodule

`default_nettype wire