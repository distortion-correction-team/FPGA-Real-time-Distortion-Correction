`timescale 1ns / 1ps

module euler_rotate_q16_16(
 // ------------------------------------------------------------
    // 修正后的光心坐标定义 (Q12.8)
    // CX = 632 * 256 = 161792
    // CY = 382 * 256 = 87040
    // ------------------------------------------------------------
    input signed [19:0] CX_256 ,
    input signed [19:0] CY_256 ,

    input  wire                     clk,
    input  wire                     rst_n,

    input  wire signed [31:0]       cos_roll,
    input  wire signed [31:0]       sin_roll,
    input  wire signed [31:0]       cos_pitch,
    input  wire signed [31:0]       sin_pitch,
    input  wire signed [31:0]       cos_yaw,
    input  wire signed [31:0]       sin_yaw,

    input  wire signed [19:0]       src_x,
    input  wire signed [19:0]       src_y,
    input  wire                     de_in,

    output reg signed [19:0]       target_x,
    output reg signed [19:0]       target_y,
    output reg                      coordinates_valid 
);


    // ------------------------------------------------------------
    // 流水线控制 (延迟严格为 6 拍)
    // ------------------------------------------------------------
    localparam LATENCY = 6;
    reg [LATENCY-1:0] valid_pipe;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid_pipe <= {LATENCY{1'b0}};
        else
            valid_pipe <= {valid_pipe[LATENCY-2:0], de_in};
    end
    wire out_valid = valid_pipe[LATENCY-1];

    // ------------------------------------------------------------
    // Stage 1 : 输入打拍 + 去中心化
    // ------------------------------------------------------------
    reg signed [31:0] s1_cr, s1_sr, s1_cp, s1_sp, s1_cy, s1_sy;
    reg signed [19:0] s1_x,  s1_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_cr <= 0; s1_sr <= 0; s1_cp <= 0; s1_sp <= 0; s1_cy <= 0; s1_sy <= 0;
            s1_x  <= 0; s1_y  <= 0;
        end else if (de_in) begin
            s1_cr <= cos_roll;  s1_sr <= sin_roll;
            s1_cp <= cos_pitch; s1_sp <= sin_pitch;
            s1_cy <= cos_yaw;   s1_sy <= sin_yaw;
            // 去中心化：src - 632 / 340
            s1_x  <= src_x - CX_256; 
            s1_y  <= src_y - CY_256;
        end
    end

    // ------------------------------------------------------------
    // Stage 2 : 基础矩阵元素乘法 + 坐标打拍
    // ------------------------------------------------------------
    reg signed [63:0] p_cy_cp, p_sy_cp;
    reg signed [63:0] p_cy_cr, p_sy_cr;
    reg signed [63:0] p_cy_sp, p_sy_sp;
    reg signed [19:0] s2_x, s2_y;
    reg signed [31:0] s2_sr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_cy_cp <= 0; p_sy_cp <= 0; p_cy_cr <= 0; p_sy_cr <= 0; p_cy_sp <= 0; p_sy_sp <= 0;
            s2_x <= 0; s2_y <= 0; s2_sr <= 0;
        end else begin
            p_cy_cp <= s1_cy * s1_cp;
            p_sy_cp <= s1_sy * s1_cp;
            p_cy_cr <= s1_cy * s1_cr;
            p_sy_cr <= s1_sy * s1_cr;
            p_cy_sp <= s1_cy * s1_sp;
            p_sy_sp <= s1_sy * s1_sp;
            s2_x <= s1_x;
            s2_y <= s1_y;
            s2_sr <= s1_sr;
        end
    end

    // ------------------------------------------------------------
    // Stage 3 : 四舍五入与 Q16.16 转换
    // ------------------------------------------------------------
    localparam signed [63:0] ROUND_CONST = 64'sd32768; // 2^15

    wire signed [31:0] q_cy_cp = (p_cy_cp + ROUND_CONST) >>> 16;
    wire signed [31:0] q_sy_cp = (p_sy_cp + ROUND_CONST) >>> 16;
    wire signed [31:0] q_cy_sp = (p_cy_sp + ROUND_CONST) >>> 16;
    wire signed [31:0] q_sy_sp = (p_sy_sp + ROUND_CONST) >>> 16;
    wire signed [31:0] q_cy_cr = (p_cy_cr + ROUND_CONST) >>> 16;
    wire signed [31:0] q_sy_cr = (p_sy_cr + ROUND_CONST) >>> 16;

    reg signed [31:0] r11, r21;
    reg signed [31:0] s3_cy_sp, s3_sy_sp, s3_sr, s3_cy_cr, s3_sy_cr;
    reg signed [19:0] s3_x, s3_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r11 <= 0; r21 <= 0;
            s3_cy_sp <= 0; s3_sy_sp <= 0; s3_sr <= 0; s3_cy_cr <= 0; s3_sy_cr <= 0;
            s3_x <= 0; s3_y <= 0;
        end else begin
            r11 <= q_cy_cp;
            r21 <= q_sy_cp;
            s3_cy_sp <= q_cy_sp;
            s3_sy_sp <= q_sy_sp;
            s3_sr    <= s2_sr;
            s3_cy_cr <= q_cy_cr;
            s3_sy_cr <= q_sy_cr;
            s3_x <= s2_x;
            s3_y <= s2_y;
        end
    end

    // ------------------------------------------------------------
    // Stage 4 : 形成 R12/R22 并进行坐标乘法
    // ------------------------------------------------------------
    wire signed [31:0] r12_wire = (((s3_cy_sp * s3_sr) + ROUND_CONST) >>> 16) - s3_sy_cr;
    wire signed [31:0] r22_wire = (((s3_sy_sp * s3_sr) + ROUND_CONST) >>> 16) + s3_cy_cr;

    reg signed [51:0] prod_x1, prod_x2;
    reg signed [51:0] prod_y1, prod_y2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_x1 <= 0; prod_x2 <= 0; prod_y1 <= 0; prod_y2 <= 0;
        end else begin
            prod_x1 <= r11 * s3_x;
            prod_x2 <= r12_wire * s3_y;
            prod_y1 <= r21 * s3_x;
            prod_y2 <= r22_wire * s3_y;
        end
    end

    // ------------------------------------------------------------
    // Stage 5 : 累加、四舍五入并右移 16 位得到 Q12.8
    // ------------------------------------------------------------
    reg signed [36:0] shifted_x, shifted_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shifted_x <= 0; shifted_y <= 0;
        end else begin
            shifted_x <= (prod_x1 + prod_x2 + 53'sd32768) >>> 16;
            shifted_y <= (prod_y1 + prod_y2 + 53'sd32768) >>> 16;
        end
    end

    // ------------------------------------------------------------
    // Stage 6 : 回中心化 + 越界判断 (纯组合逻辑)
    // ------------------------------------------------------------
    // 加回中心坐标 (632, 340)
    wire signed [36:0] final_x = shifted_x + CX_256;
    wire signed [36:0] final_y = shifted_y + CY_256;

    localparam signed [36:0] X_MIN = 37'sd0;
    localparam signed [36:0] X_MAX = 37'sd327679; // 1280*256 - 1
    localparam signed [36:0] Y_MIN = 37'sd0;
    localparam signed [36:0] Y_MAX = 37'sd184319; // 720*256 - 1

    // 越界判断直接基于回中心化后的真实坐标
    wire w_out_of_bounds = (final_x < X_MIN || final_x > X_MAX || final_y < Y_MIN || final_y > Y_MAX);

    // ------------------------------------------------------------
    // Stage 7 : 输出寄存器
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_x          <= 0;
            target_y          <= 0;
            coordinates_valid <= 1'b0;
        end else begin
            // 将最终坐标截断为 Q12.8 输出
            target_x          <= final_x[19:0];
            target_y          <= final_y[19:0];
            coordinates_valid <= out_valid & ~w_out_of_bounds; 
        end
    end

endmodule