`default_nettype none
// tt_top.v — TinyTapeout adapter for chip_core (pure pin glue; all logic
// lives in chip_core.v). D7 rev2 pinout:
//   ui_in[7:0]         = beat in (NOP=0x00 idle)
//   {uio[7:4], uo_out} = 12b output frame (count-deterministic offsets;
//                        frame_valid intentionally unpinned, see [D7-2])
//   uio[0]=CS_N, uio[1]=MOSI, uio[2]=MISO(in), uio[3]=SCK
//   uio_oe = 8'b1111_1011 STATIC. rst_n active-low -> rst = ~rst_n. [TT-3]

module tt_um_sky_tpu_3x3 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    wire        rst = ~rst_n;
    wire [11:0] frame;
    wire        frame_valid;
    wire        cs_n_w, sck_w, mosi_w;

    chip_core core (
        .clk(clk), .rst(rst),
        .beat_in(ui_in),
        .frame(frame), .frame_valid(frame_valid),
        .spi_cs_n(cs_n_w), .spi_sck(sck_w),
        .spi_mosi(mosi_w), .spi_miso(uio_in[2])
    );

    assign uo_out  = frame[7:0];
    assign uio_out = {frame[11:8], sck_w, 1'b0, mosi_w, cs_n_w};
    assign uio_oe  = 8'b1111_1011;

    wire _unused = &{ena, uio_in[7:3], uio_in[1:0], frame_valid, 1'b0};

endmodule

`default_nettype wire