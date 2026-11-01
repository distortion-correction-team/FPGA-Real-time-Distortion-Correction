`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 
// Design Name: 
// Module Name: gray_convert
// Project Name: HDMI_DDR3_OV5640
// Target Devices: Pango PG2L100H
// Tool Versions: PDS
// Description: RGB565 to grayscale converter, 4-stage pipeline.
//              Y = 0.299*R + 0.587*G + 0.114*B
//
//   Input and output are both RGB565. The gray value is written into the
//   R, G and B channels at the same time, so the downstream 565->888
//   expansion and the MS72xx timing need no modification at all.
//
//   Timing: de / vs / hs travel through the very same pipeline as the pixel
//   data. Output is 4 pix_clk cycles behind the input, and the three sync
//   signals stay aligned with the data. This alignment is essential: never
//   delay the data without delaying de as well.
//
//   Algorithm: first expand the 5/6/5 bit components to 8 bit by bit
//   replication (more accurate than zero padding), then apply the fixed
//   point coefficients 77 / 150 / 29 and shift right by 8:
//       Y8 = (77*R8 + 150*G8 + 29*B8) >> 8
//   The three coefficients are built from shift-and-add terms only, so no
//   DSP block is required and a pure LUT adder tree easily meets the
//   37.125 MHz output pixel clock.
//
// Dependencies: none
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//   gray_en = 1 -> grayscale output; gray_en = 0 -> original color bypass
//   (handy for on-board A/B comparison).
//   If gray_en comes from a push button or any other asynchronous source,
//   synchronize it to pix_clk before feeding this module.
// 
//////////////////////////////////////////////////////////////////////////////////
`define UD #1

module gray_convert (
    input               pix_clk     ,   // pixel clock, tie to top level pix_clk (37.125MHz)
    input               rst_n       ,   // active low reset, tie to ddr_init_done
    input               gray_en     ,   // 1 = grayscale, 0 = color bypass

    //---------------- video input (from fram_buf read side) ----------------
    input               de_i        ,   // data enable
    input               vs_i        ,   // vertical sync (pass through)
    input               hs_i        ,   // horizontal sync (pass through)
    input      [15:0]   rgb_i       ,   // RGB565 {R[4:0],G[5:0],B[4:0]}

    //---------------- video output (to 565->888 expansion) -----------------
    output reg          de_o        ,
    output reg          vs_o        ,
    output reg          hs_o        ,
    output reg [15:0]   rgb_o
);

    //==========================================================================
    // stage 0 : input register + RGB565 -> RGB888 bit expansion
    //==========================================================================
    reg        de_s0, vs_s0, hs_s0;
    reg [15:0] rgb_s0;
    reg [7:0]  r8_s0, g8_s0, b8_s0;

    //==========================================================================
    // stage 1 : coefficient multiply (shift-and-add, coeff 77 / 150 / 29)
    //==========================================================================
    reg [15:0] r_w_s1, g_w_s1, b_w_s1;
    reg        de_s1, vs_s1, hs_s1;
    reg [15:0] rgb_s1;

    //==========================================================================
    // stage 2 : sum / shift / pack into gray RGB565
    //==========================================================================
    reg        de_s2, vs_s2, hs_s2;
    reg [15:0] rgb_s2;                  // color bypass channel
    reg [15:0] gray_s2;                 // grayscale channel

    // 77  = 64 + 8 + 4 + 1
    wire [15:0] r_term = (r8_s0 << 6) + (r8_s0 << 3) + (r8_s0 << 2) + r8_s0;
    // 150 = 128 + 16 + 4 + 2
    wire [15:0] g_term = (g8_s0 << 7) + (g8_s0 << 4) + (g8_s0 << 2) + (g8_s0 << 1);
    // 29  = 32 - 2 - 1
    wire [15:0] b_term = (b8_s0 << 5) - b8_s0 - (b8_s0 << 1);

    wire [15:0] y_sum   = r_w_s1 + g_w_s1 + b_w_s1;   // max 255*256 = 65280
    wire [7:0]  y8      = y_sum[15:8];
    wire [15:0] gray565 = {y8[7:3], y8[7:2], y8[7:3]};// R = G = B = gray

    //==========================================================================
    // 4-stage pipeline
    //==========================================================================
    always @(posedge pix_clk)
    begin
        if(!rst_n)
        begin
            de_s0 <= `UD 1'b0;  vs_s0 <= `UD 1'b0;  hs_s0 <= `UD 1'b0;
            rgb_s0 <= `UD 16'd0;
            r8_s0 <= `UD 8'd0;  g8_s0 <= `UD 8'd0;  b8_s0 <= `UD 8'd0;
        end
        else
        begin
            de_s0 <= `UD de_i;
            vs_s0 <= `UD vs_i;
            hs_s0 <= `UD hs_i;
            rgb_s0 <= `UD rgb_i;
            // bit replication expansion, closer to true luminance than zero padding
            r8_s0 <= `UD {rgb_i[15:11], rgb_i[13:11]};
            g8_s0 <= `UD {rgb_i[10:5],  rgb_i[6:5] };
            b8_s0 <= `UD {rgb_i[4:0],   rgb_i[2:0] };
        end
    end

    always @(posedge pix_clk)
    begin
        if(!rst_n)
        begin
            r_w_s1 <= `UD 16'd0;  g_w_s1 <= `UD 16'd0;  b_w_s1 <= `UD 16'd0;
            de_s1  <= `UD 1'b0;   vs_s1  <= `UD 1'b0;   hs_s1  <= `UD 1'b0;
            rgb_s1 <= `UD 16'd0;
        end
        else
        begin
            r_w_s1 <= `UD r_term;
            g_w_s1 <= `UD g_term;
            b_w_s1 <= `UD b_term;
            de_s1  <= `UD de_s0;
            vs_s1  <= `UD vs_s0;
            hs_s1  <= `UD hs_s0;
            rgb_s1 <= `UD rgb_s0;
        end
    end

    always @(posedge pix_clk)
    begin
        if(!rst_n)
        begin
            de_s2   <= `UD 1'b0;  vs_s2 <= `UD 1'b0;  hs_s2 <= `UD 1'b0;
            rgb_s2  <= `UD 16'd0;
            gray_s2 <= `UD 16'd0;
        end
        else
        begin
            de_s2   <= `UD de_s1;
            vs_s2   <= `UD vs_s1;
            hs_s2   <= `UD hs_s1;
            rgb_s2  <= `UD rgb_s1;
            gray_s2 <= `UD gray565;
        end
    end

    always @(posedge pix_clk)
    begin
        if(!rst_n)
        begin
            de_o  <= `UD 1'b0;  vs_o <= `UD 1'b0;  hs_o <= `UD 1'b0;
            rgb_o <= `UD 16'd0;
        end
        else
        begin
            de_o  <= `UD de_s2;
            vs_o  <= `UD vs_s2;
            hs_o  <= `UD hs_s2;
            rgb_o <= `UD gray_en ? gray_s2 : rgb_s2;
        end
    end

endmodule
