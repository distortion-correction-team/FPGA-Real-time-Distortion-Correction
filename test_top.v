module test_top #(
    

    // ========================================================
    // 行缓冲读偏移 = ROWS/2 - 16
    //   ROWS= 32 ->  0
    //   ROWS= 64 -> 16   <-- 本版（配合 line_buf_win_32 的 ROWS=64）
    //   ROWS=128 -> 48
    // ========================================================
    parameter [11:0] RG_OFFSET = 12'd16
)(
    input [31:0] K1,
    input [31:0] K2,
    input [31:0] K3,
    input [31:0] P1,
    input [31:0] P2,
    input [15:0] FOCAL_X ,
    input [15:0] FOCAL_Y ,
    input [31:0] CENTER_X,
    input [31:0] CENTER_Y,
    input signed [19:0] CX_256,
    input signed [19:0] CY_256,
    input [31:0] inv_focal_x ,
    input [31:0] inv_focal_y ,

    input               pix_clk     ,
    input               rst_n       ,

    input               de_i        ,
    input      [15:0]   rgb_i       ,

    input      [11:0]   h_count     ,
    input      [11:0]   v_count     ,
    input  wire signed [31:0]       cos_roll,
    input  wire signed [31:0]       sin_roll,
    input  wire signed [31:0]       cos_pitch,
    input  wire signed [31:0]       sin_pitch,
    input  wire signed [31:0]       cos_yaw,
    input  wire signed [31:0]       sin_yaw,

    output reg          de_o        ,
    output reg          vs_o        ,
    output reg          hs_o        ,
    output reg [15:0]   rgb_o
);

    localparam V_TOTAL = 12'd750;
    localparam V_SYNC  = 12'd5;
    localparam V_BP    = 12'd20;
    localparam V_ACT   = 12'd720;

    localparam H_TOTAL = 12'd1650;
    localparam H_SYNC  = 12'd40;
    localparam H_BP    = 12'd220;
    localparam H_ACT   = 12'd1280;

    // ========================================================
    // 1. 写侧坐标计数器
    // ========================================================
    reg [11:0] wr_x, wr_y;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_x <= 12'd0;
            wr_y <= 12'd0;
        end else if (de_i) begin
            if (wr_x == 12'd1279) begin
                wr_x <= 12'd0;
                if (wr_y == 12'd719)
                    wr_y <= 12'd0;
                else
                    wr_y <= wr_y + 12'd1;
            end else begin
                wr_x <= wr_x + 12'd1;
            end
        end
    end

    // ========================================================
    // 2. 读侧坐标及同步信号
    // ========================================================
    reg start_flag;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)
            start_flag <= 1'b0;
        else if (v_count == 12'd15 & h_count == 12'd1649)
            start_flag <= 1'b1;
    end

    wire [11:0] v_cnt_dly = start_flag ?
                            ((v_count >= 12'd16) ? (v_count - 12'd16)
                                                 : (v_count - 12'd16 + V_TOTAL)) : 12'd0;

    wire [11:0] v_cnt_rd = (v_cnt_dly >= RG_OFFSET) ?
                           (v_cnt_dly - RG_OFFSET) :
                           (v_cnt_dly - RG_OFFSET + V_TOTAL);

    wire [11:0] x_src = ( ( h_count < H_SYNC + H_BP + H_ACT) &
                          (h_count   >= (H_SYNC + H_BP)) ) ?
                        ( h_count - (H_SYNC + H_BP) ) : 12'd0;

    wire [11:0] y_u = (v_cnt_dly >= V_SYNC + V_BP) ?
                      (v_cnt_dly - (V_SYNC + V_BP)) : 12'd0;

    wire [11:0] y_src_raw = ((v_cnt_rd < V_SYNC + V_BP + V_ACT) &
                             (v_cnt_rd >= V_SYNC + V_BP)) ?
                            (v_cnt_rd - (V_SYNC + V_BP)) : 12'd0;

    //  黑色条带修复：顶部 RG_OFFSET 行夹到源图第 0 行
    wire [11:0] y_src = (y_u < RG_OFFSET) ? 12'd0 : y_src_raw;

    wire de_src = start_flag &&
                  (h_count >= (H_SYNC + H_BP)) && (h_count < (H_SYNC + H_BP + H_ACT)) &&
                  (v_cnt_dly >= (V_SYNC + V_BP)) && (v_cnt_dly < (V_SYNC + V_BP + V_ACT));

    wire vs_raw = start_flag && (v_cnt_dly < V_SYNC);
    wire hs_raw = start_flag && (h_count < H_SYNC);

    // ========================================================
    // 3. 坐标映射 / 欧拉旋转 / 行缓存 / 双线性插值
    // ========================================================
    wire [19:0] dist_target_x, dist_target_y;
    wire        dist_coord_valid;
       // ========================================================
    // 自适应缩放系数（Q1.15，0x8000 = 1.0）
    //   解 S + a·k1·S³ = 1，a = (cx/fx)²+(cy/fy)² = 0.3764
    //   覆盖 |k1| ≤ 1.8，14 段，每段取【区间上界】的值 ⇒ 偏保守、不留黑边
    //   误差 < 1%，约 40 个 LUT
    // ========================================================
    wire [31:0] k1_abs = K1[31] ? (~K1 + 32'd1) : K1;

    reg  [15:0] scale_q15;

    always @(*) begin
        if      (k1_abs < 32'h0000_0800) scale_q15 = 16'h8000;  // 1.0000  放大 1.00x
        else if (k1_abs < 32'h0000_1000) scale_q15 = 16'h7E8C;  // 0.9886  放大 1.01x
        else if (k1_abs < 32'h0000_2000) scale_q15 = 16'h7D2F;  // 0.9780  放大 1.02x
        else if (k1_abs < 32'h0000_3000) scale_q15 = 16'h7AB2;  // 0.9586  放大 1.04x
        else if (k1_abs < 32'h0000_4000) scale_q15 = 16'h7878;  // 0.9412  放大 1.06x
        else if (k1_abs < 32'h0000_6000) scale_q15 = 16'h7674;  // 0.9254  放大 1.08x
        
        else                             scale_q15 = 16'h7674;  // 0.7331  放大 1.36x
    end
    // ========================================================
    // 帧边界算一次 inv_f_eff = inv_f × S / 32768
    //   像素路径零代价、零 APM
    // ========================================================
    wire frame_head = (h_count == 12'd0) && (v_count == 12'd0);

    reg signed [31:0] inv_fx_eff;
    reg signed [31:0] inv_fy_eff;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            inv_fx_eff <= 32'sd56;      // 默认 = 65536/1170
            inv_fy_eff <= 32'sd56;
        end
        else if (frame_head) begin
            inv_fx_eff <= (inv_focal_x * scale_q15) >>> 15;
            inv_fy_eff <= (inv_focal_y * scale_q15) >>> 15;
        end
    end


    distortion_interpolation_ctrl u_dist_map(
        .K1         (K1      ),
        .K2         (K2      ),
        .K3         (K3      ),
        .P1         (P1      ),
        .P2         (P2      ),
        .FOCAL_X    (FOCAL_X ),
        .FOCAL_Y    (FOCAL_Y ),
        .CENTER_X   (CENTER_X),
        .CENTER_Y   (CENTER_Y),
        .inv_focal_x(inv_fx_eff),
        .inv_focal_y(inv_fy_eff),
        .pix_clk           (pix_clk),
        .rst_n             (rst_n),
        .x_src             (x_src),
        .y_src             (y_src),
        .target_x          (dist_target_x),
        .target_y          (dist_target_y),
        .coordinates_valid (dist_coord_valid)
    );

    wire signed [19:0] rot_target_x, rot_target_y;
    wire               rot_coord_valid;

    euler_rotate_q16_16 u_euler_rotate(
        .CX_256 (CX_256),
        .CY_256 (CY_256),     
        .clk               (pix_clk),
        .rst_n             (rst_n),
        .cos_roll          (cos_roll),
        .sin_roll          (sin_roll),
        .cos_pitch         (cos_pitch),
        .sin_pitch         (sin_pitch),
        .cos_yaw           (cos_yaw),
        .sin_yaw           (sin_yaw),
        .src_x             (dist_target_x),
        .src_y             (dist_target_y),
        .de_in             (dist_coord_valid),
        .target_x          (rot_target_x),
        .target_y          (rot_target_y),
        .coordinates_valid (rot_coord_valid)
    );

    wire [15:0] p00, p01, p10, p11;
    wire [7:0]  frac_x, frac_y;
    wire        buf_valid;

    line_buf_win_32 u_line_buf (
        .pix_clk           (pix_clk),
        .rst_n             (rst_n),
        .de_in             (de_i),
        .x_in              (wr_x),
        .y_in              (wr_y),
        .pixel_in          (rgb_i),
        .target_x          (rot_target_x),
        .target_y          (rot_target_y),
        .coordinates_valid (rot_coord_valid),
        .p00               (p00),
        .p01               (p01),
        .p10               (p10),
        .p11               (p11),
        .frac_x            (frac_x),
        .frac_y            (frac_y),
        .valid_out         (buf_valid)
    );

    wire [15:0] interp_rgb;
    wire        interp_valid;

    bilinear_interp u_bilinear (
        .pix_clk   (pix_clk),
        .rst_n     (rst_n),
        .p00       (p00),
        .p01       (p01),
        .p10       (p10),
        .p11       (p11),
        .frac_x    (frac_x),
        .frac_y    (frac_y),
        .valid_in  (buf_valid),
        .pixel_out (interp_rgb),
        .valid_out (interp_valid)
    );

    // ========================================================
    // 4. 输出时序对齐（19 拍 => de_dly[18]）
    // ========================================================
    reg [23:0] de_dly, vs_dly, hs_dly;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            de_dly <= 24'd0;
            vs_dly <= 24'd0;
            hs_dly <= 24'd0;
        end else begin
            de_dly <= {de_dly[22:0], de_src};
            vs_dly <= {vs_dly[22:0], vs_raw};
            hs_dly <= {hs_dly[22:0], hs_raw};
        end
    end

    // ========================================================
    // 5. 最终输出锁存
    // ========================================================
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            de_o  <= 1'b0;
            vs_o  <= 1'b0;
            hs_o  <= 1'b0;
            rgb_o <= 16'd0;
        end else begin
            de_o  <= interp_valid & de_dly[18];
            vs_o  <= vs_dly[18];
            hs_o  <= hs_dly[18];
            rgb_o <= interp_rgb;
        end
    end

endmodule