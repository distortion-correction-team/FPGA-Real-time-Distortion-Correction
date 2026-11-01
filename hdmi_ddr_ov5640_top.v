`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:Meyesemi 
// Engineer: Will
// Target Devices: Pango
// Description: HDMI_IN_DDR3_YCbCr_top —— 720p 回环 + 畸变矫正/旋转/白平衡/gamma
//              全参数经 UART 在线可配，两级运行时门控
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
//`define CMOS_1
`define CMOS_2

module hdmi_ddr_ov5640_top#(
    parameter MEM_ROW_ADDR_WIDTH   = 15         ,
    parameter MEM_COL_ADDR_WIDTH   = 10         ,
    parameter MEM_BADDR_WIDTH      = 3          ,
    parameter MEM_DQ_WIDTH         =  32        ,
    parameter MEM_DQS_WIDTH        =  32/8      ,

    parameter [31:0] K1 = 32'h0000_722A,                          
    parameter [31:0] K2 = 32'hFFFF_C5DD,                          
    parameter [31:0] K3 = 32'h0000_0000,                          
    parameter [31:0] P1 = 32'hFFFF_FFF9,                          
    parameter [31:0] P2 = 32'hFFFF_FFCD,                          
    parameter [15:0] FOCAL_X = 16'd1170,                          
    parameter [15:0] FOCAL_Y = 16'd1169,                          
    parameter [31:0] CENTER_X = 32'd632,                          
    parameter [31:0] CENTER_Y = 32'd340,                          
    parameter signed [19:0] CX_256 = 20'sd161792,                 
    parameter signed [19:0] CY_256 = 20'sd87040,                  
    parameter [31:0] INV_FOCAL_X = 32'h0000_0038,//焦距的倒数          
    parameter [31:0] INV_FOCAL_Y = 32'h0000_0038//                

)(
	input                                sys_clk              ,//27MHz
    input                                clk_p ,
    input                                clk_n ,
//OV5640
    output  [1:0]                        cmos_init_done       ,
    //coms1	
    inout                                cmos1_scl            ,
    inout                                cmos1_sda            ,
    input                                cmos1_vsync          ,
    input                                cmos1_href           ,
    input                                cmos1_pclk           ,
    input   [7:0]                        cmos1_data           ,
    output                               cmos1_reset          ,
    //coms2
    inout                                cmos2_scl            ,
    inout                                cmos2_sda            ,
    input                                cmos2_vsync          ,
    input                                cmos2_href           ,
    input                                cmos2_pclk           ,
    input   [7:0]                        cmos2_data           ,
    output                               cmos2_reset          ,
//DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
//====================================================================
    input                                uart_rx                   ,
    output                               uart_tx                   ,
    input                                corr_en_sw                ,//一级门控按键：1=启用畸变/旋转
    input                                all_en_sw                 ,//二级门控按键：1=启用全部处理
//====================================================================
//MS72xx       
    output                               rstn_out                  ,
    output                               iic_tx_scl                ,
    inout                                iic_tx_sda                ,
    output                               hdmi_int_led              ,
//HDMI_OUT
    output                               pix_clk                   ,
    output     reg                       vs_out                    , 
    output     reg                       hs_out                    , 
    output     reg                       de_out                    ,
    output     reg[7:0]                  r_out                     , 
    output     reg[7:0]                  g_out                     , 
    output     reg[7:0]                  b_out         
);

/////////////////////////////////////////////////////////////////////////////////////
// 两级门控延迟（改分辨率/改流水级数时同步改这里）
//   一级：从 o_rgb565_d1（白平衡后）延到 o_rgb565_d19 ⇒ 19 拍
//   二级：从 o_rgb565   （fram_buf 原始）延到 o_rgb565_d20 ⇒ 3+19+1 = 23 拍
/////////////////////////////////////////////////////////////////////////////////////
    localparam CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;
    localparam TH_1S    = 25'd33000000;
    localparam integer BYP1_LAT = 19;
    localparam integer BYP2_LAT = 23;

/////////////////////////////////////////////////////////////////////////////////////
    reg  [15:0]                 rstn_1ms            ;
    wire                        cmos_scl            ;
    wire                        cmos_sda            ;
    wire                        cmos_vsync          ;
    wire                        cmos_href           ;
    wire                        cmos_pclk           ;
    wire   [7:0]                cmos_data           ;
    wire                        cmos_reset          ;
    wire                        initial_en          ;
    wire[15:0]                  cmos1_d_16bit       ;
    wire                        cmos1_href_16bit    ;
    reg [7:0]                   cmos1_d_d0          ;
    reg                         cmos1_href_d0       ;
    reg                         cmos1_vsync_d0      ;
    wire                        cmos1_pclk_16bit    ;
    wire[15:0]                  cmos2_d_16bit       /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_href_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    reg [7:0]                   cmos2_d_d0          /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_href_d0       /*synthesis PAP_MARK_DEBUG="1"*/;
    reg                         cmos2_vsync_d0      /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        cmos2_pclk_16bit    /*synthesis PAP_MARK_DEBUG="1"*/;
    wire[15:0]                  o_rgb565            ;
    wire                        pclk_in_test        ;    
    wire                        vs_in_test          ;
    wire                        de_in_test          ;
    wire[15:0]                  i_rgb565   /*synthesis PAP_MARK_DEBUG="1"*/;
    wire                        de_re               ;
    wire                        init_done           ;

    // ★ 补声明 8：sync_vg 的三根输出线
    wire                        vs_o                ;
    wire                        hs_o                ;
    wire                        de_o                ;

//axi bus   
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;
    wire                        axi_awvalid                ;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;
    wire                        axi_arvalid                ;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   /* synthesis syn_keep = 1 */;
    wire                        axi_rvalid                  /* synthesis syn_keep = 1 */;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [24:0]                 cnt                        ;

    wire                        locked                     ;
    wire                        cfg_clk                    ;
    wire                        clk_25M                    ;
    wire                        init_over_rx               ;
    wire                        pll_lock                   ;
    wire                        core_clk                   ;
    wire                        phy_pll_lock               ;
    wire                        gpll_lock                  ;
    wire                        rst_gpll_lock              ;
    wire                        ddrphy_cpd_lock            ;

/////////////////////////////////////////////////////////////////////////////////////
// PLL
/////////////////////////////////////////////////////////////////////////////////////
    pll u_pll (
        .clkin1   (  sys_clk    ),
        .clkout0  (  pix_clk    ),//37.125M 720P
        .clkout1  (  cfg_clk    ),//10M
        .clkout2  (  clk_25M    ),
        .lock     (  locked     )
    );

    ms72xx_ctl ms72xx_ctl(
        .clk             (  cfg_clk        ),
        .rst_n           (  rstn_out       ),
        .init_over       (  init_over_tx   ),
        .init_over_rx    (  init_over_rx   ),
        .iic_scl         (  iic_tx_scl     ),
        .iic_sda         (  iic_tx_sda     )
    );

    wire init_over_tx;

    always @(posedge cfg_clk) begin
    	if(!locked)
    	    rstn_1ms <= 16'd0;
    	else if(rstn_1ms == 16'h2710)
    	    rstn_1ms <= rstn_1ms;
    	else
    	    rstn_1ms <= rstn_1ms + 1'b1;
    end

    assign rstn_out = (rstn_1ms == 16'h2710);

/////////////////////////////////////////////////////////////////////////////////////
// 配置 CMOS
/////////////////////////////////////////////////////////////////////////////////////
    power_on_delay	power_on_delay_inst(
    	.clk_50M                 (sys_clk        ),
    	.reset_n                 (1'b1           ),
    	.camera1_rstn            (cmos1_reset    ),
    	.camera2_rstn            (cmos2_reset    ),
    	.camera_pwnd             (               ),
    	.initial_en              (initial_en     )
    );

    reg_config	coms1_reg_config(
    	.clk_25M                 (clk_25M            ),
    	.camera_rstn             (cmos1_reset        ),
    	.initial_en              (initial_en         ),
    	.i2c_sclk                (cmos1_scl          ),
    	.i2c_sdat                (cmos1_sda          ),
    	.reg_conf_done           (cmos_init_done[0]  ),
    	.reg_index               (                   ),
    	.clock_20k               (                   )
    );

    reg_config	coms2_reg_config(
    	.clk_25M                 (clk_25M            ),
    	.camera_rstn             (cmos2_reset        ),
    	.initial_en              (initial_en         ),
    	.i2c_sclk                (cmos2_scl          ),
    	.i2c_sdat                (cmos2_sda          ),
    	.reg_conf_done           (cmos_init_done[1]  ),
    	.reg_index               (                   ),
    	.clock_20k               (                   )
    );

/////////////////////////////////////////////////////////////////////////////////////
// CMOS 8bit -> 16bit
/////////////////////////////////////////////////////////////////////////////////////
    always@(posedge cmos1_pclk) begin
        cmos1_d_d0        <= cmos1_data    ;
        cmos1_href_d0     <= cmos1_href    ;
        cmos1_vsync_d0    <= cmos1_vsync   ;
    end

    cmos_8_16bit cmos1_8_16bit(
    	.pclk           (cmos1_pclk       ),
    	.rst_n          (cmos_init_done[0]),
    	.pdata_i        (cmos1_d_d0       ),
    	.de_i           (cmos1_href_d0    ),
    	.vs_i           (cmos1_vsync_d0   ),
    	.pixel_clk      (cmos1_pclk_16bit ),
    	.pdata_o        (cmos1_d_16bit    ),
    	.de_o           (cmos1_href_16bit )
    );

    always@(posedge cmos2_pclk) begin
        cmos2_d_d0        <= cmos2_data    ;
        cmos2_href_d0     <= cmos2_href    ;
        cmos2_vsync_d0    <= cmos2_vsync   ;
    end

    cmos_8_16bit cmos2_8_16bit(
    	.pclk           (cmos2_pclk       ),
    	.rst_n          (cmos_init_done[1]),
    	.pdata_i        (cmos2_d_d0       ),
    	.de_i           (cmos2_href_d0    ),
    	.vs_i           (cmos2_vsync_d0   ),
    	.pixel_clk      (cmos2_pclk_16bit ),
    	.pdata_o        (cmos2_d_16bit    ),
    	.de_o           (cmos2_href_16bit )
    );

/////////////////////////////////////////////////////////////////////////////////////
// 视频源选择
/////////////////////////////////////////////////////////////////////////////////////
`ifdef CMOS_1
    assign pclk_in_test = cmos1_pclk_16bit ;
    assign vs_in_test   = cmos1_vsync_d0   ;
    assign de_in_test   = cmos1_href_16bit ;
    assign i_rgb565     = {cmos1_d_16bit[4:0],cmos1_d_16bit[10:5],cmos1_d_16bit[15:11]};
`elsif CMOS_2
    assign pclk_in_test = cmos2_pclk_16bit ;
    assign vs_in_test   = cmos2_vsync_d0   ;
    assign de_in_test   = cmos2_href_16bit ;
    assign i_rgb565     = {cmos2_d_16bit[4:0],cmos2_d_16bit[10:5],cmos2_d_16bit[15:11]};
`endif
    assign hdmi_int_led = |i_rgb565;

/////////////////////////////////////////////////////////////////////////////////////
// DDR 帧缓冲
/////////////////////////////////////////////////////////////////////////////////////
    fram_buf fram_buf(
        .ddr_clk        (  core_clk             ),
        .ddr_rstn       (  ddr_init_done        ),
        .vin_clk        (  pclk_in_test         ),
        .wr_fsync       (  vs_in_test           ),
        .wr_en          (  de_in_test           ),
        .wr_data        (  i_rgb565             ),
        .vout_clk       (  pix_clk              ),
        .rd_fsync       (  vs_o                 ),
        .rd_en          (  de_re                ),
        .vout_de        (  de_o                 ),
        .vout_data      (  o_rgb565             ),
        .init_done      (  init_done            ),
        .axi_awaddr     (  axi_awaddr           ),
        .axi_awid       (  axi_awuser_id        ),
        .axi_awlen      (  axi_awlen            ),
        .axi_awsize     (                       ),
        .axi_awburst    (                       ),
        .axi_awready    (  axi_awready          ),
        .axi_awvalid    (  axi_awvalid          ),
        .axi_wdata      (  axi_wdata            ),
        .axi_wstrb      (  axi_wstrb            ),
        .axi_wlast      (  axi_wusero_last      ),
        .axi_wvalid     (                       ),
        .axi_wready     (  axi_wready           ),
        .axi_bid        (  4'd0                 ),
        .axi_araddr     (  axi_araddr           ),
        .axi_arid       (  axi_aruser_id        ),
        .axi_arlen      (  axi_arlen            ),
        .axi_arsize     (                       ),
        .axi_arburst    (                       ),
        .axi_arvalid    (  axi_arvalid          ),
        .axi_arready    (  axi_arready          ),
        .axi_rready     (                       ),
        .axi_rdata      (  axi_rdata            ),
        .axi_rvalid     (  axi_rvalid           ),
        .axi_rlast      (  axi_rlast            ),
        .axi_rid        (  axi_rid              )
    );

/////////////////////////////////////////////////////////////////////////////////////
// 视频时序
/////////////////////////////////////////////////////////////////////////////////////
    wire [11:0] x_act;
    wire [11:0] y_act;
    wire        de_o_0;
    wire [11:0] h_count_i;
    wire [11:0] v_count_i;

    sync_vg sync_vg(
        .clk            (  pix_clk              ),
        .rstn           (  ddr_init_done        ),
        .vs_out         (  vs_o                 ),
        .hs_out         (  hs_o                 ),
        .de_out         (  de_re                ),
        .de_re          (  de_o_0               ),
        .x_act          (  x_act                ),
        .y_act          (  y_act                ),
        .h_count        (  h_count_i            ),
        .v_count        (  v_count_i            )
    );

//====================================================================
// 白平衡
//====================================================================
    wire [15:0] o_rgb565_d1;
    wire        hs_o_d1;
    wire        vs_o_d1;
    wire        de_o_d1;
    wire [7:0]  bright;
    wire        vsync_down;

    white_balance u_white_balance(
        .pix_clk       (pix_clk),
        .rst_n         (ddr_init_done),
        .pix_in        (o_rgb565),
        .vsync         (vs_o),
        .hsync         (hs_o),
        .de            (de_o),
        .pix_out       (o_rgb565_d1),
        .hsync_out     (hs_o_d1),
        .vsync_out     (vs_o_d1),
        .de_out        (de_o_d1),
        .bright        (bright),
        .vsync_down_out(vsync_down)
    );

//====================================================================
// 串口收帧 + CORDIC（★ 修复 1/2/3/6/7）
//====================================================================
    // ★ 修复 6：这些都接 datapack_ctrl 的 output，必须是 wire，不能是 reg
    wire signed [31:0] ROLL_cos, ROLL_sin;
    wire signed [31:0] PITCH_cos, PITCH_sin;
    wire signed [31:0] YAW_cos,   YAW_sin;

    //wire signed [31:0] ROLL_rt, PITCH_rt, YAW_rt;

    wire signed [31:0] K1_rt, K2_rt, K3_rt, P1_rt, P2_rt;
    wire [15:0] FOCAL_X_rt, FOCAL_Y_rt;
    wire [31:0] CENTER_X_rt, CENTER_Y_rt;
    wire [31:0] inv_focal_x_rt, inv_focal_y_rt;

    //串口出来的门控改名，避免与顶层 input 端口重名
    wire        corr_en_uart;
    wire        all_en_uart;

    //四个 valid 声明
    wire        calc_valid;
    wire        sw_valid;
    wire        distort_valid;
    wire        camera_valid;

    
    datapack_ctrl u_datapack_ctrl(
        .clk           (pix_clk),
        .rst_n         (ddr_init_done),
        .uart_rx       (uart_rx),

        //----- 角度 sin/cos -----
        .ROLL_cos      (ROLL_cos ),
        .ROLL_sin      (ROLL_sin ),
        .PITCH_cos     (PITCH_cos),
        .PITCH_sin     (PITCH_sin),
        .YAW_cos       (YAW_cos  ),
        .YAW_sin       (YAW_sin  ),
        //----- 角度原值（CORDIC 输入）-----
        //.ROLL          (ROLL_rt  ),
        // .PITCH         (PITCH_rt ),
        //.YAW           (YAW_rt   ),
        //----- 畸变 -----
        .K1            (K1_rt),
        .K2            (K2_rt),
        .K3            (K3_rt),
        .P1            (P1_rt),
        .P2            (P2_rt),
        //----- 内参 -----
        .FOCAL_X       (FOCAL_X_rt    ),
        .FOCAL_Y       (FOCAL_Y_rt    ),
        .CENTER_X      (CENTER_X_rt   ),
        .CENTER_Y      (CENTER_Y_rt   ),
        .inv_focal_x   (inv_focal_x_rt),
        .inv_focal_y   (inv_focal_y_rt),
        //----- 门控 -----
        .corr_en_sw    (corr_en_uart  ),
        .all_en_sw     (all_en_uart   ),
        //----- 串口 -----
        .uart_tx       (uart_tx       ),
        .calc_valid    (calc_valid    ),
        .sw_valid      (sw_valid      ),
        .distort_valid (distort_valid ),
        .camera_valid  (camera_valid  )
    );

//====================================================================
// valid 只有 1 拍，用累积标志等到 vsync 再消费
//====================================================================
    reg pend_sw, pend_dist, pend_cam;

    always @(posedge pix_clk or negedge ddr_init_done) begin
        if(!ddr_init_done) begin
            pend_sw   <= 1'b0;
            pend_dist <= 1'b0;
            pend_cam  <= 1'b0;
        end
        else begin
            // 先消费（vsync 边沿）
            if(vsync_down) begin
                if(pend_sw)   pend_sw   <= 1'b0;
                if(pend_dist) pend_dist <= 1'b0;
                if(pend_cam)  pend_cam  <= 1'b0;
            end
            // 再置位（后写优先；若同拍到达则保留到下一帧）
            if(sw_valid)      pend_sw   <= 1'b1;
            if(distort_valid) pend_dist <= 1'b1;
            if(camera_valid)  pend_cam  <= 1'b1;
        end
    end

//====================================================================
// 参数影子寄存器（vsync 下降沿整体切换，帧内恒定）
//====================================================================
    reg signed [31:0] ROLL_cos_s,  ROLL_sin_s;
    reg signed [31:0] PITCH_cos_s, PITCH_sin_s;
    reg signed [31:0] YAW_cos_s,   YAW_sin_s;

    reg signed [31:0] K1_s, K2_s, K3_s, P1_s, P2_s;
    reg [15:0] FOCAL_X_s, FOCAL_Y_s;
    reg [31:0] CENTER_X_s, CENTER_Y_s;
    reg [31:0] inv_focal_x_s, inv_focal_y_s;

    reg        corr_en_s;
    reg        all_en_s;

    always @(posedge pix_clk or negedge ddr_init_done) begin
        if(!ddr_init_done) begin
            // 角度默认：无旋转
            ROLL_cos_s  <= 32'sd65536;  ROLL_sin_s  <= 32'sd0;
            PITCH_cos_s <= 32'sd65536;  PITCH_sin_s <= 32'sd0;
            YAW_cos_s   <= 32'sd65536;  YAW_sin_s   <= 32'sd0;
            // 畸变 / 内参：用顶层 parameter 作默认值
            K1_s <= K1;   K2_s <= K2;   K3_s <= K3;   P1_s <= P1;   P2_s <= P2;
            FOCAL_X_s     <= FOCAL_X;
            FOCAL_Y_s     <= FOCAL_Y;
            CENTER_X_s    <= {16'd0, CENTER_X};
            CENTER_Y_s    <= {16'd0, CENTER_Y};
            inv_focal_x_s <= INV_FOCAL_X;
            inv_focal_y_s <= INV_FOCAL_Y;
            // 门控默认全开
            corr_en_s <= 1'b1;
            all_en_s  <= 1'b1;
        end
        else if(vsync_down) begin
            // 改成并行 if，同一帧可同时更新多组参数
            if(calc_valid) begin
                ROLL_cos_s  <= ROLL_cos ;
                ROLL_sin_s  <= ROLL_sin ;
                PITCH_cos_s <= PITCH_cos;
                PITCH_sin_s <= PITCH_sin;
                YAW_cos_s   <= YAW_cos  ;
                YAW_sin_s   <= YAW_sin  ;
            end
            if(pend_sw) begin
                corr_en_s <= corr_en_uart & corr_en_sw;   // 串口 & 按键
                all_en_s  <= all_en_uart  & all_en_sw;
            end
            if(pend_dist) begin
                K1_s <= K1_rt;
                K2_s <= K2_rt;
                K3_s <= K3_rt;
                P1_s <= P1_rt;
                P2_s <= P2_rt;
            end
            if(pend_cam) begin
                FOCAL_X_s     <= FOCAL_X_rt;
                FOCAL_Y_s     <= FOCAL_Y_rt;
                CENTER_X_s    <= CENTER_X_rt;
                CENTER_Y_s    <= CENTER_Y_rt;
                inv_focal_x_s <= inv_focal_x_rt;
                inv_focal_y_s <= inv_focal_y_rt;
            end
        end
    end

//====================================================================
// ★ 修复 10：CX_256 / CY_256 由影子中心点现算，不从串口收
//   cx 最大 1920 ⇒ ×256 = 491520 < 2^19 ⇒ 20 位足够
//====================================================================
    wire signed [19:0] CX_256_s = CENTER_X_s[11:0] << 8;
    wire signed [19:0] CY_256_s = CENTER_Y_s[11:0] << 8;

//====================================================================
// 畸变 + 旋转 + 双线性插值（★ 修复 9：参数全部接影子寄存器）
//====================================================================
    wire        de_o_d19;
    wire        vs_o_d19;
    wire        hs_o_d19;
    wire [15:0] o_rgb565_d19;

    test_top u_distortion(
        .K1           (K1_s          ),
        .K2           (K2_s          ),
        .K3           (K3_s          ),
        .P1           (P1_s          ),
        .P2           (P2_s          ),
        .FOCAL_X      (FOCAL_X_s     ),
        .FOCAL_Y      (FOCAL_Y_s     ),
        .CENTER_X     (CENTER_X_s    ),
        .CENTER_Y     (CENTER_Y_s    ),
        .CX_256       (CX_256_s      ),
        .CY_256       (CY_256_s      ),
        .inv_focal_x  (inv_focal_x_s ),
        .inv_focal_y  (inv_focal_y_s ),
    
        .pix_clk    (pix_clk),
        .rst_n      (ddr_init_done),

        .de_i       (de_o_d1),
        .rgb_i      (o_rgb565_d1),

        .h_count    (h_count_i),
        .v_count    (v_count_i),

        .cos_roll   (ROLL_cos_s ),
        .sin_roll   (ROLL_sin_s ),
        .cos_pitch  (PITCH_cos_s),
        .sin_pitch  (PITCH_sin_s),
        .cos_yaw    (YAW_cos_s  ),
        .sin_yaw    (YAW_sin_s  ),

        .de_o       (de_o_d19),
        .vs_o       (vs_o_d19),
        .hs_o       (hs_o_d19),
        .rgb_o      (o_rgb565_d19)
    );

//====================================================================
// 一级门控：畸变/旋转 开关（19 拍等长旁路）
//====================================================================
    reg [BYP1_LAT*16-1:0] byp_rgb_sr;

    always @(posedge pix_clk or negedge ddr_init_done) begin
        if (!ddr_init_done)
            byp_rgb_sr <= {(BYP1_LAT*16){1'b0}};
        else
            byp_rgb_sr <= { byp_rgb_sr[BYP1_LAT*16-17 : 0], o_rgb565_d1 };
    end

    wire [15:0] o_rgb565_byp = byp_rgb_sr[BYP1_LAT*16-1 : (BYP1_LAT-1)*16];
    wire [15:0] o_rgb565_sel = corr_en_s ? o_rgb565_d19 : o_rgb565_byp;

//====================================================================
// gamma
//====================================================================
    reg [18:0] vsync_down_r;
    wire       vsync_down_d19;

    always @(posedge pix_clk) begin
        if (!ddr_init_done)
            vsync_down_r <= 19'd0;
        else
            vsync_down_r <= {vsync_down_r[17:0], vsync_down};
    end

    assign vsync_down_d19 = vsync_down_r[18];

    wire        de_o_d20;
    wire        vs_o_d20;
    wire        hs_o_d20;
    wire [15:0] o_rgb565_d20;

    gamma_correct u_gamma_correct(
        .pix_clk     (pix_clk),
        .rst_n       (ddr_init_done),
        .pix_in      (o_rgb565_sel),
        .de          (de_o_d19),
        .hsync       (hs_o_d19),
        .vsync       (vs_o_d19),
        .bright      (bright),
        .vsync_down  (vsync_down_d19),
        .pix_out     (o_rgb565_d20),
        .de_out      (de_o_d20),
        .hsync_out   (hs_o_d20),
        .vsync_out   (vs_o_d20)
    );

//====================================================================
// 二级门控：全部处理 开关（23 拍等长旁路）
//====================================================================
    reg [BYP2_LAT*16-1:0] raw_rgb_sr;

    always @(posedge pix_clk or negedge ddr_init_done) begin
        if (!ddr_init_done)
            raw_rgb_sr <= {(BYP2_LAT*16){1'b0}};
        else
            raw_rgb_sr <= { raw_rgb_sr[BYP2_LAT*16-17 : 0], o_rgb565 };
    end

    wire [15:0] o_rgb565_raw = raw_rgb_sr[BYP2_LAT*16-1 : (BYP2_LAT-1)*16];
    wire [15:0] o_rgb565_out = all_en_s ? o_rgb565_d20 : o_rgb565_raw;

//====================================================================
// 输出寄存器
//====================================================================
    always@(posedge pix_clk) begin
        r_out  <= {o_rgb565_out[15:11],3'b0};
        g_out  <= {o_rgb565_out[10:5], 2'b0};
        b_out  <= {o_rgb565_out[4:0],  3'b0};
        vs_out <= vs_o_d20;
        hs_out <= hs_o_d20;
        de_out <= de_o_d20;
    end

/////////////////////////////////////////////////////////////////////////////////////
// DDR3
/////////////////////////////////////////////////////////////////////////////////////
    wire clk_125Mhz;

    GTP_INBUFGDS #(
        .IOSTANDARD("DEFAULT"),
        .TERM_DIFF("ON")
    ) u_gtp (
        .O(clk_125Mhz),
        .I(clk_p),
        .IB(clk_n)
    );

    ddr3_test u_ddr3_test_h(
        .ref_clk                   (clk_125Mhz            ),
        .resetn                    (rstn_out              ),
        .ddr_init_done             (ddr_init_done         ),
        .pll_lock                  (pll_lock              ),
        .core_clk                  (core_clk              ),
        .phy_pll_lock              (phy_pll_lock          ),
        .gpll_lock                 (gpll_lock             ),
        .rst_gpll_lock             (rst_gpll_lock         ),
        .ddrphy_cpd_lock           (ddrphy_cpd_lock       ),

        .axi_awaddr                (axi_awaddr            ),
        .axi_awuser_ap             (1'b0                  ),
        .axi_awuser_id             (axi_awuser_id         ),
        .axi_awlen                 (axi_awlen             ),
        .axi_awready               (axi_awready           ),
        .axi_awvalid               (axi_awvalid           ),
        .axi_wdata                 (axi_wdata             ),
        .axi_wstrb                 (axi_wstrb             ),
        .axi_wready                (axi_wready            ),
        .axi_wusero_id             (                      ),
        .axi_wusero_last           (axi_wusero_last       ),
        .axi_araddr                (axi_araddr            ),
        .axi_aruser_ap             (1'b0                  ),
        .axi_aruser_id             (axi_aruser_id         ),
        .axi_arlen                 (axi_arlen             ),
        .axi_arready               (axi_arready           ),
        .axi_arvalid               (axi_arvalid           ),
        .axi_rdata                 (axi_rdata             ),
        .axi_rid                   (axi_rid               ),
        .axi_rlast                 (axi_rlast             ),
        .axi_rvalid                (axi_rvalid            ),

        .apb_clk                   (1'b0                  ),
        .apb_rst_n                 (1'b1                  ),
        .apb_sel                   (1'b0                  ),
        .apb_enable                (1'b0                  ),
        .apb_addr                  (8'b0                  ),
        .apb_write                 (1'b0                  ),
        .apb_ready                 (                      ),
        .apb_wdata                 (16'b0                 ),
        .apb_rdata                 (                      ),

        .mem_rst_n                 (mem_rst_n             ),
        .mem_ck                    (mem_ck                ),
        .mem_ck_n                  (mem_ck_n              ),
        .mem_cke                   (mem_cke               ),
        .mem_cs_n                  (mem_cs_n              ),
        .mem_ras_n                 (mem_ras_n             ),
        .mem_cas_n                 (mem_cas_n             ),
        .mem_we_n                  (mem_we_n              ),
        .mem_odt                   (mem_odt               ),
        .mem_a                     (mem_a                 ),
        .mem_ba                    (mem_ba                ),
        .mem_dqs                   (mem_dqs               ),
        .mem_dqs_n                 (mem_dqs_n             ),
        .mem_dq                    (mem_dq                ),
        .mem_dm                    (mem_dm                ),

        .dbg_gate_start(1'b0),
        .dbg_cpd_start(1'b0),
        .dbg_ddrphy_rst_n(1'b1),
        .dbg_gpll_scan_rst(1'b0),
        .samp_position_dyn_adj(1'b0),
        .init_samp_position_even(32'd0),
        .init_samp_position_odd(32'd0),
        .wrcal_position_dyn_adj(1'b0),
        .init_wrcal_position(32'd0),
        .force_read_clk_ctrl(1'b0),
        .init_slip_step(16'd0),
        .init_read_clk_ctrl(12'd0),
        .debug_calib_ctrl(),
        .dbg_slice_status(),
        .dbg_slice_state(),
        .debug_data(),
        .dbg_dll_upd_state(),
        .debug_gpll_dps_phase(),
        .dbg_rst_dps_state(),
        .dbg_tran_err_rst_cnt(),
        .dbg_ddrphy_init_fail(),
        .debug_cpd_offset_adj(1'b0),
        .debug_cpd_offset_dir(1'b0),
        .debug_cpd_offset(10'd0),
        .debug_dps_cnt_dir0(),
        .debug_dps_cnt_dir1(),
        .ck_dly_en(1'b0),
        .init_ck_dly_step(8'h0),
        .ck_dly_set_bin(),
        .align_error(),
        .debug_rst_state(),
        .debug_cpd_state()
    );

/////////////////////////////////////////////////////////////////////////////////////
// 心跳
/////////////////////////////////////////////////////////////////////////////////////
    always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 25'd0;
        else if (cnt >= TH_1S)
            cnt <= 25'd0;
        else
            cnt <= cnt + 25'd1;
    end

    always @(posedge core_clk) begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if (cnt >= TH_1S)
            heart_beat_led <= ~heart_beat_led;
    end

endmodule