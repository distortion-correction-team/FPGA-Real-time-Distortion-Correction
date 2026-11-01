`timescale 1ns / 1ps

module datapack_ctrl(
    input  clk,                 // 37.125 MHz
    input  rst_n,
    input  uart_rx,
//=================角度================
    output reg signed [31:0] ROLL_cos,
    output reg signed [31:0] ROLL_sin,
    output reg signed [31:0] PITCH_cos,
    output reg signed [31:0] PITCH_sin,
    output reg signed [31:0] YAW_cos,
    output reg signed [31:0] YAW_sin,
    output reg signed [31:0] ROLL,
    output reg signed [31:0] PITCH,
    output reg signed [31:0] YAW,
//=================畸变================
    output reg signed [31:0] K1            ,
    output reg signed [31:0] K2            ,
    output reg signed [31:0] K3            ,
    output reg signed [31:0] P1            ,
    output reg signed [31:0] P2            ,
//=================内参================
    output reg [15:0] FOCAL_X       ,
    output reg [15:0] FOCAL_Y       ,
    output reg [31:0] CENTER_X      ,//接受的是16位，补0调整为32位
    output reg [31:0] CENTER_Y      ,
               
    output reg [31:0] inv_focal_x   ,
    output reg [31:0] inv_focal_y   ,
//=================门控================
    output reg          corr_en_sw    ,
    output reg          all_en_sw     ,

//=================串口================
    output      uart_tx,
    output reg  calc_valid,
    output reg  sw_valid,
    output reg  distort_valid,
    output reg  camera_valid
);

    //==============================================================
    // 帧解析状态机
    //==============================================================
    localparam IDLE = 4'd0, HEAD = 4'd1,
               ANGLE = 4'd2, SW = 4'd3, DISTORT = 4'd4,
               CAMERA = 4'd5, CHECK = 4'd6, TAIL = 4'd7;

    reg  [3:0] state, next_state;
    wire [7:0] rx_data;
    wire       rx_valid_byte;

    reg rx_valid;
    reg rx_valid_byte_d;
    always @(posedge clk) begin
        if(!rst_n) rx_valid_byte_d <= 1'b0;
        else       rx_valid_byte_d <= rx_valid_byte;
    end
    wire rx_valid_R = rx_valid_byte & ~rx_valid_byte_d;

    
    reg [7:0] data_cnt;
    reg [7:0] XOR;
    reg [7:0] cmd;

    always @(posedge clk) begin
        state <= next_state;                        // 保持：next_state 不变则 state 不变
        if(!rst_n) begin
            state      <= 4'd0;
            next_state <= 4'd0;
        end
        else if(rx_valid_R) begin
            case(state)
                IDLE : next_state <= (rx_data == 8'hAA) ? HEAD : IDLE;
                HEAD : begin
                        if(rx_data == 8'h06)begin
                            next_state<=ANGLE;
                            cmd<=8'h06;
                        end
                        else if(rx_data == 8'h05)begin
                            next_state<=CAMERA;
                            cmd<=8'h05;
                        end
                        else if(rx_data == 8'h04)begin
                            next_state<=DISTORT;
                            cmd<=8'h04;
                        end
                        else if(rx_data == 8'h02)begin
                            next_state<=SW;
                            cmd<=8'h02;
                        end
                        else begin
                            next_state<=IDLE;
                            cmd<=8'h00;
                        end
                       end
                ANGLE  : next_state <= (data_cnt == 8'd104 ) ? CHECK : ANGLE;
                SW     : next_state <= CHECK;
                DISTORT: next_state <= (data_cnt == 8'd152) ? CHECK : DISTORT;
                CAMERA : next_state <= (data_cnt == 8'd120) ? CHECK : CAMERA;
                CHECK  : next_state <= (rx_data == XOR) ? TAIL : IDLE;
                TAIL   : next_state <= IDLE;
                default: next_state <= IDLE;
            endcase
        end
        
    end

    reg [111:0]  angle;
    reg [7:0]   sw;
    reg [159:0] distort;
    reg [127:0] camera;

    always @(posedge clk) begin
        if(!rst_n) begin
            data_cnt <= 8'd0;
            angle     <= 112'd0;
            sw        <= 8'd3;
            distort   <= 160'hFFFF_FFCD_FFFF_FFF9_0000_0000_FFFF_C5DD_0000_122A;
            camera    <= 128'h0000_0038_0000_0038_0154_0278_0491_0492;
            XOR       <= 8'd0;
            rx_valid  <= 1'b0;
            camera_valid  <= 1'b0;
            distort_valid <= 1'b0;
            sw_valid      <= 1'b0;
        end
        else if(rx_valid_R) begin
            case(state)
                IDLE: begin
                    angle     <=112'd0;
                    sw        <= 8'd3;
                    distort   <= 160'hFFFF_FFCD_FFFF_FFF9_0000_0000_FFFF_C5DD_0000_122A;
                    camera    <= 128'h0000_0038_0000_0038_0154_0278_0491_0492;
                    data_cnt <= 8'd0;
                    rx_valid  <= 1'b0;
                    camera_valid   <=  1'b0;
                    distort_valid  <=  1'b0;
                    sw_valid       <=  1'b0;
                    if (rx_data == 8'hAA) XOR <= 8'hAA;
                end
                HEAD:  XOR <= XOR ^ rx_data;

                ANGLE: begin
                    if(data_cnt == 8'd104) data_cnt <= 8'd0;
                    else                  data_cnt <= data_cnt + 8'd8;
                    angle[data_cnt +: 8] <= rx_data;
                    XOR <= XOR ^ rx_data;
                end

                SW: begin
                    data_cnt<=8'd0;
                    sw[7:0]<=rx_data;
                    XOR <= XOR ^ rx_data;
                end

                DISTORT: begin
                    if(data_cnt == 8'd152) data_cnt <= 8'd0;
                    else                   data_cnt <= data_cnt + 8'd8;
                    distort[data_cnt +: 8] <= rx_data;
                    XOR <= XOR ^ rx_data;
                end

                CAMERA: begin
                    if(data_cnt == 8'd120) data_cnt <= 8'd0;
                    else                   data_cnt <= data_cnt + 8'd8;
                    camera[data_cnt +: 8] <= rx_data;
                    XOR <= XOR ^ rx_data;
                end

                TAIL: begin
                    if(rx_data == 8'h55)begin
                        case(cmd)
                            8'h06:rx_valid       <=  1'b1;
                            8'h05:camera_valid   <=  1'b1;
                            8'h04:distort_valid  <=  1'b1;
                            8'h02:sw_valid       <=  1'b1;
                            default:;
                        endcase
                    end
                    else                 rx_valid <= 1'b0;
                end
            endcase
        end
    end

    //==============================================================
    // 角度字段缓冲（只响应 0x06）
    //==============================================================
    always @(posedge clk) begin
        if(!rst_n) begin
            ROLL  <= 32'd0;
            PITCH <= 32'd0;
            YAW   <= 32'd0;
        end
        else if(rx_valid) begin
            ROLL  <= angle[31:0];
            PITCH <= angle[63:32];
            YAW   <= angle[95:64];
        end
    end

    //==============================================================
    // 畸变输出（只响应 0x04，Q16.16）
    //==============================================================
    always @(posedge clk) begin
        if(!rst_n) begin
             K1  <= 32'h0000_122A;
             K2  <= 32'hFFFF_C5DD;
             K3  <= 32'h0000_0000;
             P1  <= 32'hFFFF_FFF9;
             P2  <= 32'hFFFF_FFCD;
        end
        else if(distort_valid) begin
             K1  <= distort[31:0]   ;
             K2  <= distort[63:32]  ;
             K3  <= distort[95:64]  ;
             P1  <= distort[127:96] ;
             P2  <= distort[159:128];
        end
    end

    //==============================================================
    // 内参输出（只响应 0x05）
    //   camera[ 15:  0] = FOCAL_X      (16 bit)
    //   camera[ 31: 16] = FOCAL_Y      (16 bit)
    //   camera[ 47: 32] = CENTER_X     (16 bit)
    //   camera[ 63: 48] = CENTER_Y     (16 bit)
    //   camera[ 95: 64] = INV_FOCAL_X  (32 bit, Q16)
    //   camera[127: 96] = INV_FOCAL_Y  (32 bit, Q16)
    //==============================================================
    always @(posedge clk) begin
        if(!rst_n) begin
            FOCAL_X     <= 16'd1170;
            FOCAL_Y     <= 16'd1169;
            CENTER_X    <= 32'd632;
            CENTER_Y    <= 32'd340;
            inv_focal_x <= 32'h0000_0038;   // = 65536/1170 = 56
            inv_focal_y <= 32'h0000_0038;
        end
        else if(camera_valid) begin
            FOCAL_X     <= camera[15:0]   ;
            FOCAL_Y     <= camera[31:16]  ;
            CENTER_X    <= camera[47:32]  ;
            CENTER_Y    <= camera[63:48]  ;
            inv_focal_x <= camera[95:64]  ;
            inv_focal_y <= camera[127:96] ;
        end
    end

    //==============================================================
    // 门控输出（只响应 0x02）
    //==============================================================
    always @(posedge clk) begin
        if(!rst_n) begin
             corr_en_sw <= 1'b1;
             all_en_sw  <= 1'b1;
        end
        else if(sw_valid) begin
             corr_en_sw <= sw[0];
             all_en_sw  <= sw[1];
        end
    end


    //==============================================================
    //   单 CORDIC 时分复用（替代 3 个）
    //==============================================================

    // ---- (1) 触发脉冲：等 ROLL/PITCH/YAW 写入完成后再启动 ----
    reg rx_valid_f1, rx_valid_f2;
    always @(posedge clk) begin
        if(!rst_n) begin
            rx_valid_f1 <= 1'b0;
            rx_valid_f2 <= 1'b0;
        end
        else begin
            rx_valid_f1 <= rx_valid;
            rx_valid_f2 <= rx_valid_f1;
        end
    end
    wire trig = rx_valid_f1 & ~rx_valid_f2;      // 严格 1 拍

    // ---- (2) 送数相位 0/1/2 ----
    reg [1:0] ph;
    reg       feeding;

    always @(posedge clk) begin
        if(!rst_n) begin
            ph      <= 2'd0;
            feeding <= 1'b0;
        end
        else if(trig) begin
            ph      <= 2'd1;
            feeding <= 1'b1;
        end
        else if(feeding) begin
            if(ph == 2'd2) begin
                ph      <= 2'd0;
                feeding <= 1'b0;
            end
            else begin
                ph <= ph + 2'd1;
            end
        end
    end

    wire in_valid = trig | feeding;

    // ---- (3) 3:1 输入选择 ----
    wire signed [31:0] theta_mux = (ph == 2'd0) ? ROLL  :
                                   (ph == 2'd1) ? PITCH : YAW;

    // ---- (4) 寄存一级：CORDIC 输入前是纯寄存器 ----
    reg signed [31:0] theta_r;
    reg               in_valid_r;
    reg [1:0]         ph_r;

    always @(posedge clk) begin
        if(!rst_n) begin
            theta_r    <= 32'sd0;
            in_valid_r <= 1'b0;
            ph_r       <= 2'd0;
        end
        else begin
            theta_r    <= theta_mux;
            in_valid_r <= in_valid;
            ph_r       <= ph;
        end
    end

    // ---- (5) 相位标签延迟链：18 级 x 2bit ----
    //   与 CORDIC 实际 latency 严格对应：1(初值) + 16(迭代) + 1(输出) = 18
    localparam integer TAG_LAT = 18;
    localparam integer TAG_W   = 2*TAG_LAT;      // 36

    reg [TAG_W-1:0] ph_sr;
    always @(posedge clk) begin
        if(!rst_n) ph_sr <= {TAG_W{1'b0}};
        else       ph_sr <= {ph_sr[TAG_W-3:0], ph_r};
    end
    wire [1:0] ph_out = ph_sr[TAG_W-1 -: 2];

    // ---- (6) 唯一的 CORDIC（模块本身不改）----
    wire signed [31:0] cordic_cos, cordic_sin;
    wire               cordic_valid;

    CORDIC u_cordic (
        .clk      (clk),
        .rst_n    (rst_n),
        .theta    (theta_r),
        .rx_valid (in_valid_r),
        .cos      (cordic_cos),
        .sin      (cordic_sin),
        .valid_out(cordic_valid)
    );

    //==============================================================
    // (7) 第一段：只做暂存，不对外
    //==============================================================
    reg signed [31:0] sq_rl_cos, sq_rl_sin;
    reg signed [31:0] sq_pt_cos, sq_pt_sin;
    reg signed [31:0] sq_ya_cos, sq_ya_sin;

    always @(posedge clk) begin
        if(!rst_n) begin
            sq_rl_cos <= 32'sd0;  sq_rl_sin <= 32'sd0;
            sq_pt_cos <= 32'sd0;  sq_pt_sin <= 32'sd0;
            sq_ya_cos <= 32'sd0;  sq_ya_sin <= 32'sd0;
        end
        else if(cordic_valid) begin
            case(ph_out)
                2'd0: begin sq_rl_cos <= cordic_cos; sq_rl_sin <= cordic_sin; end
                2'd1: begin sq_pt_cos <= cordic_cos; sq_pt_sin <= cordic_sin; end
                default: begin sq_ya_cos <= cordic_cos; sq_ya_sin <= cordic_sin; end
            endcase
        end
    end

    //==============================================================
    // (8) commit 脉冲：YAW 暂存完成后 1 拍
    //==============================================================
    reg commit;
    always @(posedge clk) begin
        if(!rst_n) commit <= 1'b0;
        else       commit <= cordic_valid & (ph_out == 2'd2);
    end

    //==============================================================
    // (9) 原子提交：6 个输出【同一拍】一起更新
    //==============================================================
    always @(posedge clk) begin
        if(!rst_n) begin
            ROLL_cos   <= 32'sd65536;  ROLL_sin   <= 32'sd0;
            PITCH_cos  <= 32'sd65536;  PITCH_sin  <= 32'sd0;
            YAW_cos    <= 32'sd65536;  YAW_sin    <= 32'sd0;
            calc_valid <= 1'b0;
        end
        else if(commit) begin
            ROLL_cos   <= sq_rl_cos;   ROLL_sin   <= sq_rl_sin;
            PITCH_cos  <= sq_pt_cos;   PITCH_sin  <= sq_pt_sin;
            YAW_cos    <= sq_ya_cos;   YAW_sin    <= sq_ya_sin;
            calc_valid <= 1'b1;                  //   电平，此后恒为 1
        end
    end

    //==============================================================
    // UART
    //==============================================================
    UART_top u_UART_top(
        .clk      (clk),
        .uart_rx  (uart_rx),
        .rst_n    (rst_n),
        .tx_en    (1'b0),           
        .rx_valid (rx_valid_byte),
        .rx_data  (rx_data),
        .uart_tx  (uart_tx)
    );

endmodule