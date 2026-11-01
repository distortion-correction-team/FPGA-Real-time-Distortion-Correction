module tx(
    input  wire       clk,
    input  wire       rst_n,        // 异步复位，低有效
    input  wire       baud_tick,    // 波特率脉冲（来自 baud_gen）
    input  wire       tx_start,     // 启动发送（1 拍脉冲）
    input  wire [7:0] tx_data,      // 待发送字节
    output reg        txd,          // 串行输出线
    output reg        tx_busy,      // 忙标志
    output reg        tx_done       // 发送完成（1 拍脉冲）
);

localparam S_IDLE  = 2'd0,
           S_START = 2'd1,
           S_DATA  = 2'd2,
           S_STOP  = 2'd3;

reg [1:0] state;
reg [2:0] bit_cnt;      // 数据位计数 0~7
reg [7:0] shift_reg;    // 移位寄存器

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        txd <= 1'b1;
        tx_busy <= 1'b0;
        tx_done <= 1'b0;
        state <= S_IDLE;
        bit_cnt <= 3'd0;
        shift_reg <= 8'd0;
    end
    else begin  // else 在always内部！！
        tx_done <= 1'b0;
        if(baud_tick) begin
            case(state)
                S_IDLE: begin
                    txd <= 1'b1;
                    if(tx_start) begin
                        shift_reg <= tx_data;
                        tx_busy <= 1'b1;
                        state <= S_START;
                    end
                end
                S_START: begin
                    txd <= 1'b0;
                    state <= S_DATA;
                end
                S_DATA: begin
                    shift_reg <= {1'b0, shift_reg[7:1]};
                    txd <= shift_reg[0];
                    if(bit_cnt == 3'd7) begin
                        state <= S_STOP;
                        bit_cnt <= 3'd0;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
                S_STOP: begin
                    tx_done <= 1'b1;
                    tx_busy <= 1'b0;
                    txd <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
end

endmodule
