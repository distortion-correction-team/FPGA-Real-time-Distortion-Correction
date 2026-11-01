module CORDIC(
    input                    clk,
    input                    rst_n,
    input      signed [31:0] theta,        //   补全位宽：Q16.16 弧度
    input                    rx_valid,    //外部接收数据包指示信号
    output reg signed [31:0] cos,          //   cos = x[16]
    output reg signed [31:0] sin,          //   sin = y[16]
    output reg               valid_out
);

    localparam K     = 32'd107922;         // 1.646760258 @Q16.16（备用）
    localparam inv_K = 32'd39797;          // 0.607252935 @Q16.16    初值
    localparam W     = 32;                 //   不是 6'd32

    
    // arctan 查找表
    function signed [31:0] arctan_lut(input integer idx);
        begin
            case(idx)
                0: arctan_lut = 32'sd51472;
                1: arctan_lut = 32'sd30386;
                2: arctan_lut = 32'sd16055;
                3: arctan_lut = 32'sd8150;
                4: arctan_lut = 32'sd4091;
                5: arctan_lut = 32'sd2047;
                default: arctan_lut = 32'sd1024 >>> (idx - 6);  // i>=6 → 2^(16-i)
            endcase
        end
    endfunction   
    // 流水寄存器：x/y/z/vld，共 0..16 共 17 组

    reg signed [W-1:0] x [0:16];
    reg signed [W-1:0] y [0:16];
    reg signed [W-1:0] z [0:16];
    reg                vld [0:16];

    //--------------------------------------------------------------
    // 初值级（第 0 拍）
    //      关键修复：z[0] = theta，不是 0
    //--------------------------------------------------------------
    always @(posedge clk) begin
        if(!rst_n) begin
            x[0]   <= inv_K;               // 1/K，把总增益提前补偿掉
            y[0]   <= 32'sd0;
            z[0]   <= 32'sd0;
            vld[0] <= 1'b0;
        end
        else begin
            x[0]   <= inv_K;
            y[0]   <= 32'sd0;
            z[0]   <= theta;               //     输入角度
            vld[0] <= rx_valid;
        end
    end

    //--------------------------------------------------------------
    // 16 级迭代（每级 1 拍）
    //--------------------------------------------------------------
    genvar i;
    generate
      for(i = 0; i < 16; i = i + 1) begin : ITER

        
        wire d = z[i][W-1];

        
        wire signed [W-1:0] xs = y[i] >>> i;   // 给 x 用
        wire signed [W-1:0] ys = x[i] >>> i;   //   给 y 用（用 x 的移位）

        always @(posedge clk) begin
            if(!rst_n) begin
                x[i+1]   <= 32'sd0;
                y[i+1]   <= 32'sd0;
                z[i+1]   <= 32'sd0;
                vld[i+1] <= 1'b0;
            end
            else begin
                x[i+1]   <= d ? (x[i] + xs) : (x[i] - xs);
                y[i+1]   <= d ? (y[i] - ys) : (y[i] + ys);
                z[i+1]   <= d ? (z[i] + arctan_lut(i)) : (z[i] - arctan_lut(i));
                vld[i+1] <= vld[i];
            end
        end

      end
    endgenerate

    
    // 输出级（第 18 拍）
    //     cos 来自 x[16]，sin 来自 y[16]
    always @(posedge clk) begin
        if(!rst_n) begin
            cos       <= 32'sd0;
            sin       <= 32'sd0;
            valid_out <= 1'b0;
        end
        else begin
            cos       <= x[16];
            sin       <= y[16];
            valid_out <= vld[16];
        end
    end

endmodule