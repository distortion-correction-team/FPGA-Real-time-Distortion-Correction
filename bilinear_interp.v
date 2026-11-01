module bilinear_interp (
    input  wire        pix_clk,
    input  wire        rst_n,
    input  wire [15:0] p00, p01, p10, p11,
    input  wire [7:0]  frac_x, frac_y,   // 去掉多余的分号
    input  wire        valid_in,
    output reg  [15:0] pixel_out,
    output reg         valid_out
);
    
    // 1. 拆分 RGB565 分量
    wire [4:0] r00 = p00[15:11], r01 = p01[15:11], r10 = p10[15:11], r11 = p11[15:11];
    wire [5:0] g00 = p00[10:5],  g01 = p01[10:5],  g10 = p10[10:5],  g11 = p11[10:5];
    wire [4:0] b00 = p00[4:0],   b01 = p01[4:0],   b10 = p10[4:0],   b11 = p11[4:0];

    // 2. 权重 (扩展成 9 位无符号数，防止 256 溢出，计算总和为 256*256)
    wire [8:0] inv_x = 9'd256 - {1'b0, frac_x};  // (256 - frac_x)
    wire [8:0] inv_y = 9'd256 - {1'b0, frac_y};  // (256 - frac_y)
    wire [8:0] f_x   = {1'b0, frac_x};
    wire [8:0] f_y   = {1'b0, frac_y};

    wire [17:0] w00 = inv_x * inv_y; // 9位 * 9位 = 18位
    wire [17:0] w01 = f_x   * inv_y;
    wire [17:0] w10 = inv_x * f_y;
    wire [17:0] w11 = f_x   * f_y;

    // 3. 加权求和 (最大 31 * 65536 ≈ 2M，需要 21 位，转成 32 位算不溢出)
    wire [31:0] sum_r = r00*w00 + r01*w01 + r10*w10 + r11*w11;
    wire [31:0] sum_g = g00*w00 + g01*w01 + g10*w10 + g11*w11;
    wire [31:0] sum_b = b00*w00 + b01*w01 + b10*w10 + b11*w11;

    // 4. 流水线输出寄存器
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_out <= 16'd0;
            valid_out <= 1'b0;
        end
        else begin
            // 右移 16 位，截取有效位 (除以 65536)
            // R 和 B 是 5 位，取 [20:16]；G 是 6 位，取 [21:16]
            pixel_out <= { 
                sum_r[20:16], 
                sum_g[21:16], 
                sum_b[20:16] 
            }; 
            valid_out <= valid_in;
        end
    end
endmodule