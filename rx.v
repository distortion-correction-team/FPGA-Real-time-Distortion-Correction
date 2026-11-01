module rx(
    input wire clk,
    input wire rst_n,
    input wire tick_16x,   // 16x 波特率脉冲
    input wire rxd,        // 串行输入（异步）
    output reg [7:0] rx_data, // 接收到的字节
    output reg rx_valid,   // 接收有效（1 拍脉冲）
    output reg rx_busy     // 忙标志
);

reg rxd_d0, rxd_d1;
always@(posedge clk) begin
    if(!rst_n) begin
        rxd_d0 <= 1'b1;
        rxd_d1 <= 1'b1;
    end else begin
        rxd_d0 <= rxd;
        rxd_d1 <= rxd_d0;
    end
end
wire rxd_sync = rxd_d1;

localparam S_IDLE  = 2'd0;
localparam S_START = 2'd1;
localparam S_DATA  = 2'd2;
localparam S_STOP  = 2'd3;

reg [1:0] state;
reg [3:0] sample_cnt;
reg [2:0] bit_cnt;
reg [7:0] shift_reg;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state     <= S_IDLE;
        sample_cnt<= 4'd0;
        bit_cnt   <= 3'd0;
        shift_reg <= 8'd0;
        rx_data   <= 8'd0;
        rx_valid  <= 1'b0;
        rx_busy   <= 1'b0;
    end else begin
        rx_valid <= 1'b0;
        if(tick_16x) begin
            case(state)
                S_IDLE: begin
                    sample_cnt <= 4'd0;
                    bit_cnt    <= 3'd0;
                    rx_busy    <= 1'b0;
                    if(rxd_sync == 1'b0) begin
                        state     <= S_START;
                        rx_busy   <= 1'b1;
                        sample_cnt<= 4'd0;
                    end
                end

                S_START: begin
                    if(sample_cnt == 4'd7) begin
                        sample_cnt <= 4'd0;
                        if(rxd_sync == 1'b0) begin
                            state <= S_DATA;
                            bit_cnt <= 3'd0;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        sample_cnt <= sample_cnt + 4'd1;
                    end
                end

                S_DATA: begin
                    if(sample_cnt == 4'd15) begin
                        sample_cnt <= 4'd0;
                        shift_reg <= {rxd_sync, shift_reg[7:1]};
                        if(bit_cnt == 3'd7) begin
                            bit_cnt <= 3'd0;
                            state <= S_STOP;
                        end else begin
                            bit_cnt <= bit_cnt + 3'd1;
                        end
                    end else begin
                        sample_cnt <= sample_cnt + 4'd1;
                    end
                end

                S_STOP: begin
                    if(sample_cnt == 4'd15) begin
                        sample_cnt <= 4'd0;
                        rx_data  <= shift_reg;
                        rx_valid <= 1'b1;
                        rx_busy  <= 1'b0;
                        state    <= S_IDLE;
                    end else begin
                        sample_cnt <= sample_cnt + 4'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
end

endmodule
