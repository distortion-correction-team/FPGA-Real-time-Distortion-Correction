`timescale 1ns / 1ps
//==================================================================
// line_buf_win_32 —— 参数化行窗口，4 bank 零复制版
//
//   ROWS | 行对 | 每bank深度 | u_distortion DRM | 总DRM | 占用 | RG_OFFSET | 可转角度
//   -----+------+------------+------------------+-------+------+-----------+----------
//    32  |  16  |  16,384    |        32        |  44   | 28%  |     0     | +-1.43
//    64  |  32  |  32,768    |        64        |  76   | 49%  |    16     | +-2.87  <-- 本版
//   128  |  64  |  65,536    |       128        | 140   | 90%  |    48     | +-5.74
//
//   ★ 改 ROWS 后，test_top.v 的 RG_OFFSET 必须同步改为 ROWS/2 - 16
//==================================================================
module line_buf_win_32 (
    input  wire        pix_clk,
    input  wire        rst_n,

    input  wire        de_in,
    input  wire [11:0] x_in,
    input  wire [11:0] y_in,
    input  wire [15:0] pixel_in,

    input  wire [19:0] target_x,
    input  wire [19:0] target_y,
    input  wire        coordinates_valid,

    output reg  [15:0] p00, p01, p10, p11,
    output reg  [7:0]  frac_x, frac_y,
    output reg         valid_out
);

    //==================================================================
    // ★★★ 唯一需要改的参数：16 / 32 / 64 / 128
    //==================================================================
    parameter ROWS = 64;

    //------------------------------------------------------------------
    // 由 ROWS 自动推导（勿手改）
    //------------------------------------------------------------------
    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam integer RPB   = ROWS / 2;            // 行对数量
    localparam integer YW    = clog2(RPB);          // 16->4, 32->5, 64->6
    localparam integer AW    = YW + 10;             // 每 bank 地址位宽
    localparam integer DEPTH = (1 << YW) * 1024;    // 每 bank 深度（2 的幂）

    //------------------------------------------------------------------
    // 4 个 bank（PDS 合法属性：/* synthesis syn_ramstyle = "block_ram" */）
    //------------------------------------------------------------------
    reg [15:0] bk00 [0:DEPTH-1] /* synthesis syn_ramstyle = "block_ram" */;
    reg [15:0] bk01 [0:DEPTH-1] /* synthesis syn_ramstyle = "block_ram" */;
    reg [15:0] bk10 [0:DEPTH-1] /* synthesis syn_ramstyle = "block_ram" */;
    reg [15:0] bk11 [0:DEPTH-1] /* synthesis syn_ramstyle = "block_ram" */;

    //==================================================================
    // 写侧：按 (行奇偶, 列奇偶) 分拣
    //==================================================================
    wire          wr_rp   = y_in[0];
    wire          wr_cp   = x_in[0];
    wire [AW-1:0] wr_addr = { y_in[YW:1], x_in[10:1] };

    always @(posedge pix_clk) begin
        if (de_in) begin
            case ({wr_rp, wr_cp})
                2'b00: bk00[wr_addr] <= pixel_in;
                2'b01: bk01[wr_addr] <= pixel_in;
                2'b10: bk10[wr_addr] <= pixel_in;
                2'b11: bk11[wr_addr] <= pixel_in;
            endcase
        end
    end

    //==================================================================
    // 读侧地址（组合）
    //==================================================================
    wire [11:0] raw_x = target_x[19:8];
    wire [11:0] raw_y = target_y[19:8];

    wire [11:0] x_int = (raw_x > 12'd1278) ? 12'd1278 : raw_x;
    wire [11:0] y_int = (raw_y > 12'd718)  ? 12'd718  : raw_y;

    // 4 个采样点的"同奇偶代表"
    wire [12:0] xs_e = {1'b0, x_int} + {12'd0,  x_int[0]};
    wire [12:0] xs_o = {1'b0, x_int} + {12'd0, ~x_int[0]};
    wire [12:0] ys_e = {1'b0, y_int} + {12'd0,  y_int[0]};
    wire [12:0] ys_o = {1'b0, y_int} + {12'd0, ~y_int[0]};

    wire [AW-1:0] a00 = { ys_e[YW:1], xs_e[10:1] };
    wire [AW-1:0] a01 = { ys_e[YW:1], xs_o[10:1] };
    wire [AW-1:0] a10 = { ys_o[YW:1], xs_e[10:1] };
    wire [AW-1:0] a11 = { ys_o[YW:1], xs_o[10:1] };

    // 路由目标：bank 的值该去哪个 pXX
    wire [1:0] dst00 = {  y_int[0],  x_int[0] };
    wire [1:0] dst01 = {  y_int[0], ~x_int[0] };
    wire [1:0] dst10 = { ~y_int[0],  x_int[0] };
    wire [1:0] dst11 = { ~y_int[0], ~x_int[0] };

    // 【核心保护】原版逻辑，仅窗口深度随 ROWS 变
    wire signed [12:0] diff_y = $signed({1'b0,
        (y_in < 12'd100 & y_int > 12'd600) ? (y_in + 12'd720) : (y_in)}) - $signed({1'b0, y_int});
    wire safe_to_read = (diff_y >= 0) && (diff_y < ROWS);

    //==================================================================
    // 第 1 拍：同步读 4 个 bank + 控制/路由信息打拍
    //==================================================================
    reg [15:0] s00, s01, s10, s11;
    reg [1:0]  d00_1, d01_1, d10_1, d11_1;
    reg        cv_1, safe_1;
    reg [7:0]  fx_1, fy_1;

    always @(posedge pix_clk) begin
        s00 <= bk00[a00];
        s01 <= bk01[a01];
        s10 <= bk10[a10];
        s11 <= bk11[a11];

        d00_1 <= dst00;
        d01_1 <= dst01;
        d10_1 <= dst10;
        d11_1 <= dst11;

        cv_1   <= coordinates_valid;
        safe_1 <= safe_to_read;
        fx_1   <= target_x[7:0];
        fy_1   <= target_y[7:0];
    end

    //==================================================================
    // 第 2 拍：路由 mux + 输出寄存
    //==================================================================
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            p00 <= 16'h1082;  p01 <= 16'h1082;  p10 <= 16'h1082;  p11 <= 16'h1082;
            frac_x <= 8'd0;
            frac_y <= 8'd0;
            valid_out <= 1'b1;
        end
        else begin
            if (cv_1 && safe_1) begin
                p00 <= (d00_1 == 2'd0) ? s00 : (d01_1 == 2'd0) ? s01 : (d10_1 == 2'd0) ? s10 : s11;
                p01 <= (d00_1 == 2'd1) ? s00 : (d01_1 == 2'd1) ? s01 : (d10_1 == 2'd1) ? s10 : s11;
                p10 <= (d00_1 == 2'd2) ? s00 : (d01_1 == 2'd2) ? s01 : (d10_1 == 2'd2) ? s10 : s11;
                p11 <= (d00_1 == 2'd3) ? s00 : (d01_1 == 2'd3) ? s01 : (d10_1 == 2'd3) ? s10 : s11;
            end
            else begin
                p00 <= 16'h1082;  p01 <= 16'h1082;
                p10 <= 16'h1082;  p11 <= 16'h1082;
            end
            frac_x <= fx_1;
            frac_y <= fy_1;
        end
    end

endmodule