module gamma_correct (
    input  wire        pix_clk,
    input  wire        rst_n,
    input  wire [15:0] pix_in,
    input  wire        de,
    input  wire        hsync,
    input  wire        vsync,
    input  wire [7:0]  bright,
    input vsync_down,

    output wire [15:0] pix_out,
    output wire        de_out,
    output wire        hsync_out,
    output wire        vsync_out
);

    // ---------- 1. RGB565 拆 ----------
    wire [4:0] r5 = pix_in[15:11];
    wire [5:0] g6 = pix_in[10:5];
    wire [4:0] b5 = pix_in[4:0];

    // ---------- 2. 扩 8 位（位复制）----------
    wire [7:0] r8 = {r5, r5[4:2]};
    wire [7:0] g8 = {g6, g6[5:4]};
    wire [7:0] b8 = {b5, b5[4:2]};
    //根据bright进行选择
 
    wire [1:0] gamma_sel_cur;
    assign gamma_sel_cur =
    (bright <  8'd55)  ? 2'd3 :   // 暗
    (bright <  8'd105) ? 2'd2 :   // 偏暗
    (bright <  8'd165) ? 2'd1 :   // 正常
                         2'd0;    // 亮
    reg [1:0] cnt;
    reg [1:0] gamma_sel_d_v1;
    //gamma_sel延迟一帧
    always@(posedge pix_clk)begin
        if(!rst_n)begin
            gamma_sel_d_v1<=2'd1;

        end
        else if(vsync_down)begin
            gamma_sel_d_v1<=gamma_sel_cur;
        end

    end
    reg [1:0] gamma_sel;
    //计数gamma_sel是否重复达到三次
    always@(posedge pix_clk)begin
        if(!rst_n)begin
            cnt<=2'd0;
            gamma_sel<=2'd1;
        end
        else if(vsync_down)begin
            if(cnt==2'd3)begin
                cnt<=2'd0;
                gamma_sel<=gamma_sel_d_v1; 
            end
            else if(gamma_sel_d_v1 == gamma_sel_cur)begin
                cnt<=cnt+1'b1;
            end
            else if(gamma_sel_d_v1 != gamma_sel_cur)begin
                cnt<=2'b0;
        end
        end
        

    


    end
    //生成gmma_sel+r8的地址位
    wire [9:0] addr_r8 = {gamma_sel,r8};
    wire [9:0] addr_g8 = {gamma_sel,g8};
    wire [9:0] addr_b8 = {gamma_sel,b8};





    // ---------- 3. Gamma ROM（3 个，并行读）----------
    wire [7:0] r_g, g_g, b_g;
 
    gamma_rom u_rom_r (
        .clk  (pix_clk),
        .rst  (!rst_n),        // 高有效复位
        .addr (addr_r8),
        .rd_data (r_g)
    );
    gamma_rom u_rom_g (
        .clk  (pix_clk),
        .rst  (!rst_n),
        .addr (addr_g8),
        .rd_data (g_g)
    );
    gamma_rom u_rom_b (
        .clk  (pix_clk),
        .rst  (!rst_n),
        .addr (addr_b8),
        .rd_data (b_g)
    );
 
    // ---------- 4. 缩回 565 ----------
    assign pix_out = {r_g[7:3], g_g[7:2], b_g[7:3]};

    // ---------- 5. 同步信号延迟 1 拍（和 LUT 对齐）----------
    reg de_d1, hs_d1, vs_d1;
    always @(posedge pix_clk) begin
        de_d1 <= de;
        hs_d1 <= hsync;
        vs_d1 <= vsync;
    end
    assign de_out    = de_d1;
    assign hsync_out = hs_d1;
    assign vsync_out = vs_d1;

endmodule