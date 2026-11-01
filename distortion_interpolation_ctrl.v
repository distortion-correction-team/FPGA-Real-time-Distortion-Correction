module distortion_interpolation_ctrl #(
    parameter               X_BITS=12,
    parameter               Y_BITS=12
 )(
  //畸变矩阵，前三个是径向，后两个切向
    input [31:0] K1,
    input [31:0] K2,
    input [31:0] K3,
    input [31:0] P1,
    input [31:0] P2,
    input [15:0] FOCAL_X ,
    input [15:0] FOCAL_Y ,
    input [31:0] CENTER_X,
    input [31:0] CENTER_Y,
    input [31:0] inv_focal_x,//焦距的倒数
    input [31:0] inv_focal_y,//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!这里记得改


    input pix_clk,
    input rst_n,
    
    input [X_BITS-1:0] x_src,
    input [Y_BITS-1:0] y_src,
   
    output reg [19:0] target_x,
    output reg [19:0] target_y,
    output reg coordinates_valid

   );


    
    localparam IMG_WIDTH = 1280;
    localparam IMG_HEIGHT = 720;


    // ===================================================================
    // 内部寄存器声明 (8级流水线版本)
    // ===================================================================
 
    reg signed [31:0] x_norm, y_norm;
    reg signed [31:0] x_norm_d1, y_norm_d1;
    reg signed [31:0] r_sq;
    reg signed [31:0] x_norm_d2, y_norm_d2;
    reg signed [31:0] r_sq_d1;
    reg signed [31:0] r_4, r_6;
    reg signed [31:0] x_norm_d3, y_norm_d3;
    reg signed [31:0] radial_poly;
    reg signed [31:0] k1_r2, k2_r4, k3_r6;
    reg signed [31:0] x_norm_d4, y_norm_d4;
    reg signed [31:0] r2;
    reg signed [31:0] x_tangential, y_tangential;
    reg signed [31:0] xy_product, x2, y2, r2_plus_2x2, r2_plus_2y2;
    reg signed [79:0] x_radial_product, y_radial_product;
    reg signed [79:0] x_radial_product_d1, y_radial_product_d1;
    reg signed [31:0] x_tangential_d1, y_tangential_d1;
    reg signed [47:0] x_radial_shifted, y_radial_shifted;
    reg signed [47:0] x_distorted, y_distorted;
    reg signed [63:0] x_final, y_final;
    //输出部分 
    reg [11:0] final_x_int;
    reg [11:0] final_y_int;
    reg [7:0] final_x_frac, final_y_frac;
    reg valid;
    wire [47:0] x_int_part, y_int_part;   // 修复：加宽到48位
    wire        x_invalid, y_invalid;
    reg [11:0] valid_delay;
    
    


    // 第0级：坐标转换为相对坐标(相对光心)+归一化
    // 修复：x_src/y_src 先零扩展到32位再转 signed，防止消隐期 x_src>=2048 时
    //       bit11 被 $signed 误当符号位，导致 x_norm 完全错乱
    always@(posedge pix_clk)begin
        x_norm <= ($signed({20'd0, x_src}) - $signed(CENTER_X)) * $signed(inv_focal_x);
        y_norm <= ($signed({20'd0, y_src}) - $signed(CENTER_Y)) * $signed(inv_focal_y);
        valid_delay[0] <= (x_src < IMG_WIDTH) && (y_src < IMG_HEIGHT);
    end   
  
    //第1级：计算归一化坐标x、y的平方并求出归一化坐标距离光心的距离
    always@(posedge pix_clk)begin
        x_norm_d1 <= x_norm;
        y_norm_d1 <= y_norm;
        
        r_sq <= (($signed(x_norm) * $signed(x_norm)) >>> 16) +
                (($signed(y_norm) * $signed(y_norm)) >>> 16);
        
        valid_delay[1] <= valid_delay[0];
    end
    
    //第2级：计算r的四次方和六次方
    always@(posedge pix_clk)begin
        x_norm_d2 <= x_norm_d1;
        y_norm_d2 <= y_norm_d1;
        r_sq_d1 <= r_sq;

        r_4 = (($signed(r_sq) * $signed(r_sq)) >>> 16);
        r_6 = (($signed(r_sq) * $signed(r_4)) >>> 16);
      
        valid_delay[2] <= valid_delay[1];
    end
    
    //第3级：计算径向畸变修正项 radial_poly = (1 + k1*r^2 + k2*r^4 + k3*r^6)
    always@(posedge pix_clk)begin
        x_norm_d3 <= x_norm_d2;
        y_norm_d3 <= y_norm_d2;
        k1_r2 = (($signed(K1) * $signed(r_sq_d1)) >>> 16);
        k2_r4 = (($signed(K2) * $signed(r_4)) >>> 16);
        k3_r6 = (($signed(K3) * $signed(r_6)) >>> 16);

        radial_poly <= 32'sd65536 + $signed(k1_r2) + $signed(k2_r4) + $signed(k3_r6);
        valid_delay[3] <= valid_delay[2];
    end
    
    //第4级：计算切向畸变模型中间参数xy、x^2、y^2，顺便求解x/y_radial_product
    // 注意：r2 用非阻塞赋值，同一块内读 r2 得到的是"上一拍"的 r_sq_d1，
    //       正好与本级 x_norm_d3/x2/y2 对齐。这是有意设计，不要改成 r_sq_d1！
    always@(posedge pix_clk)begin
     
        x_norm_d4 <= x_norm_d3;
        y_norm_d4 <= y_norm_d3;
        
        r2 <= r_sq_d1;

        x_radial_product <= $signed(x_norm_d3) * $signed(radial_poly);
        y_radial_product <= $signed(y_norm_d3) * $signed(radial_poly);
        
        xy_product = (($signed(x_norm_d3) * $signed(y_norm_d3)) >>> 16);
        x2 = (($signed(x_norm_d3) * $signed(x_norm_d3)) >>> 16);
        y2 = (($signed(y_norm_d3) * $signed(y_norm_d3)) >>> 16);
        
        r2_plus_2x2 = r2 + (x2 << 1);   //阻塞赋值：同一周期内先后使用
        r2_plus_2y2 = r2 + (y2 << 1);
        
        valid_delay[4] <= valid_delay[3];
    end
    
    //第5级：计算切向畸变修正项
    // x_tang = 2*P1*xy + P2*(r^2 + 2x^2)
    // y_tang = 2*P2*xy + P1*(r^2 + 2y^2)
    // 2*P1*xy 用 >>15；2*P2*xy 用 >>14，避免显式乘2带来的位宽隐患
    always@(posedge pix_clk)begin
        x_radial_product_d1 <= x_radial_product;
        y_radial_product_d1 <= y_radial_product;
        x_tangential <= (($signed(P1) * $signed(xy_product)) >>> 15) +  
                        (($signed(P2) * $signed(r2_plus_2x2)) >>> 16);
                        
        y_tangential <= (($signed(P2) * $signed(xy_product)) >>> 15) +
                        (($signed(P1) * $signed(r2_plus_2y2)) >>> 16);
        
        valid_delay[5] <= valid_delay[4];
    end
 
    
    //第6级：计算最终结果 x/y_distorted = x/y_radial_product + x/y_tangential
    // x_radial_product 是 Q32.32，取 [63:16] 得到 Q16.16（48位 signed）
    always@(posedge pix_clk)begin
        x_radial_shifted = $signed(x_radial_product_d1[63:16]);
        y_radial_shifted = $signed(y_radial_product_d1[63:16]);
        
        x_distorted <= x_radial_shifted + $signed({{16{x_tangential[31]}}, x_tangential});
        y_distorted <= y_radial_shifted + $signed({{16{y_tangential[31]}}, y_tangential});
        
        valid_delay[6] <= valid_delay[5];
    end
    
    
    // 第7级：映射坐标
    always@(posedge pix_clk)begin
        x_final <= $signed(x_distorted) * $signed(FOCAL_X) + ($signed(CENTER_X) << 16);
        y_final <= $signed(y_distorted) * $signed(FOCAL_Y) + ($signed(CENTER_Y) << 16);
        valid_delay[7] <= valid_delay[6];
    end
   
    
    // 边界检查 (组合逻辑)
    // 修复：x_int_part/y_int_part 加宽到48位，防止 x_final 出现大正数时
    //       低16位截断后"看起来有效"，导致 x_invalid 误判
    assign x_int_part = x_final[63:16];
    assign y_int_part = y_final[63:16];
    assign x_invalid = x_final[63] || (x_int_part >= IMG_WIDTH  - 2'd2);
    assign y_invalid = y_final[63] || (y_int_part >= IMG_HEIGHT - 2'd2);
    
    always @(posedge pix_clk) begin
        if (x_invalid) begin
            final_x_int <= 12'd0;
            final_x_frac <= 8'd0;
        end else begin
            final_x_int <= (x_int_part >= IMG_WIDTH) ? (IMG_WIDTH - 1) : x_int_part[11:0];
            final_x_frac <= x_final[15:8];
        end
        
        if (y_invalid) begin
            final_y_int <= 12'd0;
            final_y_frac <= 8'd0;
        end else begin
            final_y_int <= (y_int_part >= IMG_HEIGHT) ? (IMG_HEIGHT - 1) : y_int_part[11:0];
            final_y_frac <= y_final[15:8];
        end
        
        valid <= !x_invalid && !y_invalid;
        valid_delay[8] <= valid_delay[7];
    end
    
    // 输出寄存器
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            target_x <= 0;
            target_y <= 0;
            coordinates_valid <= 0;
        end else begin
            target_x <= {final_x_int, final_x_frac};
            target_y <= {final_y_int, final_y_frac};
            coordinates_valid <= valid && valid_delay[8];
        end
    end

endmodule