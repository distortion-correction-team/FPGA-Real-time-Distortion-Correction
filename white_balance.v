`timescale 1ns / 1ps
//==================================================================
// white_balance —— 自动白平衡 + 低照度自适应降饱和
//   总延迟：3 拍（pix_in → pix_out，de/hs/vs 同步 3 拍）
//==================================================================
module white_balance(
    input  pix_clk,
    input  rst_n,                   
    input  [15:0] pix_in,
    input  vsync,
    input  hsync,
    input  de,

    output [15:0] pix_out,
    output hsync_out,
    output vsync_out,
    output de_out,
    output reg [7:0] bright,
    output vsync_down_out
);

    parameter W = 1280;
    parameter H = 720;
    parameter K = 112;

    //==================================================================
    // RGB565 拆解
    //==================================================================
    wire [4:0] r5 = pix_in[15:11];
    wire [5:0] g6 = pix_in[10:5];
    wire [4:0] b5 = pix_in[4:0];

    wire [7:0] r8 = {r5, 3'd0};
    wire [7:0] g8 = {g6, 2'd0};
    wire [7:0] b8 = {b5, 3'd0};

    //==================================================================
    // 第 1 级：控制信号打拍（与增益乘法同拍）
    //==================================================================
    reg de_d1, hsync_d1, vsync_d1;
    reg vsync_down;

    always @(posedge pix_clk) begin
        de_d1      <= de;
        hsync_d1   <= hsync;
        vsync_d1   <= vsync;
        vsync_down <= vsync_d1 && !vsync;    // 下降沿（比实际边沿晚 2 拍）
    end

    assign vsync_down_out = vsync_down;

    //==================================================================
    // 帧统计（累加）
    //==================================================================
    reg [31:0] sum_R, sum_G, sum_B;

    always @(posedge pix_clk) begin
        if(!rst_n) begin
            sum_R <= 32'd0;  sum_G <= 32'd0;  sum_B <= 32'd0;
        end
        else if(vsync_down) begin
            sum_R <= 32'd0;  sum_G <= 32'd0;  sum_B <= 32'd0;
        end
        else if(de) begin
            sum_R <= sum_R + {24'd0, r8};
            sum_G <= sum_G + {24'd0, g8};
            sum_B <= sum_B + {24'd0, b8};
        end
    end

    //==================================================================
    // 通道平均值
    //==================================================================
    wire [7:0] avg_r = sum_R[27:20];
    wire [7:0] avg_g = sum_G[27:20];
    wire [7:0] avg_b = sum_B[27:20];

    wire [7:0] avg_r_s = (avg_r < 8'd8) ? 8'd8 : avg_r;
    wire [7:0] avg_g_s = (avg_g < 8'd8) ? 8'd8 : avg_g;
    wire [7:0] avg_b_s = (avg_b < 8'd8) ? 8'd8 : avg_b;

    //==================================================================
    // 倒数 ROM（保持你的原样）
    //==================================================================
    wire [15:0] inv_avg_r, inv_avg_g, inv_avg_b;

    white_balance_table_r u_white_blance_table_r(
        .addr    (avg_r_s),
        .rd_data (inv_avg_r),
        .clk     (pix_clk),
        .rst     (!rst_n)
    );

    white_balance_table_g u_white_blance_table_g(
        .addr    (avg_g_s),
        .rd_data (inv_avg_g),
        .clk     (pix_clk),
        .rst     (!rst_n)
    );

    white_balance_table_b u_white_blance_table_b(
        .addr    (avg_b_s),
        .rd_data (inv_avg_b),
        .clk     (pix_clk),
        .rst     (!rst_n)
    );

    //==================================================================
    // 增益计算
    //     上限从 16× 降到 2.5×（163840），防止低照度噪点放大
    //     G 通道固定 1.0（以 G 为基准，只补 R/B）
    //     很暗时向 1.0 回落（gr/4 + 0.75），避免彩噪爆炸
    //==================================================================
    wire [23:0] gr_raw_w = K * inv_avg_r;
    wire [23:0] gg_raw_w = K * inv_avg_g;
    wire [23:0] gb_raw_w = K * inv_avg_b;

    wire [23:0] gr_lim = (gr_raw_w < 24'd32768)  ? 24'd32768  :      // 0.5×
                         (gr_raw_w > 24'd163840) ? 24'd163840 : gr_raw_w;  // 2.5×
    wire [23:0] gg_lim = (gg_raw_w < 24'd32768)  ? 24'd32768  :
                         (gg_raw_w > 24'd163840) ? 24'd163840 : gg_raw_w;
    wire [23:0] gb_lim = (gb_raw_w < 24'd32768)  ? 24'd32768  :
                         (gb_raw_w > 24'd163840) ? 24'd163840 : gb_raw_w;

    reg [23:0] gain_r_raw, gain_g_raw, gain_b_raw;

    always @(posedge pix_clk) begin
        if(!rst_n) begin
            gain_r_raw <= 24'd65536;
            gain_g_raw <= 24'd65536;
            gain_b_raw <= 24'd65536;
        end
        else if(vsync_down) begin
            if(bright < 8'd30) begin
                // 很暗：增益 = 0.75 + gr/4   （向 1.0 回落，避免浮点负数移位问题）
                gain_r_raw <= {2'b0, gr_lim[23:2]} + 24'd49152;
                gain_b_raw <= {2'b0, gb_lim[23:2]} + 24'd49152;
            end
            else begin
                gain_r_raw <= gr_lim;
                gain_b_raw <= gb_lim;
            end
            gain_g_raw <= 24'd65536;      //   G 固定 1.0
        end
    end

    //==================================================================
    // 应用增益（第 1 级寄存）
    //==================================================================
    reg [31:0] r32_out, g32_out, b32_out;

    always @(posedge pix_clk) begin
        r32_out <= gain_r_raw * r8;
        g32_out <= gain_g_raw * g8;
        b32_out <= gain_b_raw * b8;
    end

    wire [31:0] r_sh = r32_out >> 16;
    wire [31:0] g_sh = g32_out >> 16;
    wire [31:0] b_sh = b32_out >> 16;

    wire [7:0] r8_out = (r_sh > 32'd255) ? 8'd255 : r_sh[7:0];
    wire [7:0] g8_out = (g_sh > 32'd255) ? 8'd255 : g_sh[7:0];
    wire [7:0] b8_out = (b_sh > 32'd255) ? 8'd255 : b_sh[7:0];

    //==================================================================
    //   新增：低照度自适应降饱和
    //   X' = Y + k·(X − Y)
    //   Y  = (77R + 150G + 29B) >> 8    （系数和 = 256，可移位实现）
    //   k  = Q4.4（1.0 = 16），5 级亮度划分
    //   亮度严格不变，只压色彩浓度
    //==================================================================

    // ---- k 的 5 级划分（越暗压得越狠）----
    reg [4:0] k_tgt, k_cur;

    always @(posedge pix_clk) begin
        if(!rst_n) begin
            k_tgt <= 5'd16;
            k_cur <= 5'd16;          // 上电默认不降饱和
        end
        else begin
            if     (bright >= 8'd90) k_tgt <= 5'd16;   // 1.000   正常光
            else if(bright >= 8'd70) k_tgt <= 5'd14;   // 0.875
            else if(bright >= 8'd50) k_tgt <= 5'd13;   // 0.8125
            else if(bright >= 8'd30) k_tgt <= 5'd11;   // 0.6875
            else                     k_tgt <= 5'd10;   // 0.625   很暗

            // 斜坡：每拍最多 1 LSB，防跳变
            if     (k_cur < k_tgt) k_cur <= k_cur + 1'b1;
            else if(k_cur > k_tgt) k_cur <= k_cur - 1'b1;
        end
    end

    // ---- 降饱和第 2 级：算 Y ----
    reg [16:0] ysum;
    reg [7:0]  s1_r, s1_g, s1_b;
    reg [4:0]  s1_k;
    reg        s1_de, s1_hs, s1_vs;

    always @(posedge pix_clk) begin
        ysum  <= 17'd77 * r8_out + 17'd150 * g8_out + 17'd29 * b8_out;
        s1_r  <= r8_out;
        s1_g  <= g8_out;
        s1_b  <= b8_out;
        s1_k  <= k_cur;
        s1_de <= de_d1;          //   用 de_d1（与 r8_out 同拍），不是 de
        s1_hs <= hsync_d1;
        s1_vs <= vsync_d1;
    end

    // ---- 降饱和第 3 级：×k + 加回 Y + 饱和 ----
    wire [7:0]        y_lut = ysum[15:8];                                 // Y = ysum >> 8
    wire signed [8:0] d_r = $signed({1'b0, s1_r}) - $signed({1'b0, y_lut});
    wire signed [8:0] d_g = $signed({1'b0, s1_g}) - $signed({1'b0, y_lut});
    wire signed [8:0] d_b = $signed({1'b0, s1_b}) - $signed({1'b0, y_lut});

    wire signed [13:0] e_r = $signed({1'b0, s1_k}) * d_r;                 // 5bit × 9bit
    wire signed [13:0] e_g = $signed({1'b0, s1_k}) * d_g;
    wire signed [13:0] e_b = $signed({1'b0, s1_k}) * d_b;

    wire signed [10:0] rr = $signed({3'b0, y_lut}) + (e_r >>> 4);         // Q4.4 反缩放
    wire signed [10:0] gg = $signed({3'b0, y_lut}) + (e_g >>> 4);
    wire signed [10:0] bb = $signed({3'b0, y_lut}) + (e_b >>> 4);

    // 饱和：负 → 0，>255 → 255
    wire [7:0] rr8 = rr[10] ? 8'd0 : ((rr[9:0] > 10'd255) ? 8'd255 : rr[7:0]);
    wire [7:0] gg8 = gg[10] ? 8'd0 : ((gg[9:0] > 10'd255) ? 8'd255 : gg[7:0]);
    wire [7:0] bb8 = bb[10] ? 8'd0 : ((bb[9:0] > 10'd255) ? 8'd255 : bb[7:0]);

    //==================================================================
    // 输出寄存（第 3 拍）+ 控制信号同步到第 3 拍
    //==================================================================
    reg [15:0] pix_out_r;
    reg de_d2, hs_d2, vs_d2;

    always @(posedge pix_clk) begin
        pix_out_r <= {rr8[7:3], gg8[7:2], bb8[7:3]};
        de_d2     <= s1_de;
        hs_d2     <= s1_hs;
        vs_d2     <= s1_vs;
    end

    assign pix_out   = pix_out_r;
    assign de_out    = de_d2;
    assign hsync_out = hs_d2;
    assign vsync_out = vs_d2;

    //==================================================================
    // 亮度统计（供 gamma / 降饱和用）
    //==================================================================
    wire [31:0] y_sum = sum_R + (sum_G << 1) + sum_B;

    always @(posedge pix_clk) begin
        if(!rst_n) begin
            bright <= 8'd128;
        end
        else if(vsync_down) begin
            bright <= y_sum >> 22;
        end
    end

endmodule