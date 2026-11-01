module baud_gen(
    input  wire        clk,       // 系统时钟
    input  wire        rst_n,     // 异步复位，低有效
    input  wire [15:0] div,       // 分频除数 = Fclk / Baud
    output reg         baud_tick  // 波特率脉冲（高有效，宽 1 个 clk）
   );

    reg[15:0]cnt;

    always@(posedge clk)begin
    if(!rst_n)begin
    cnt<=16'd0;        
    baud_tick<=1'b0;
        end
    else begin
        if(cnt>=div-1'b1)begin
                baud_tick<=1'b1;
                cnt<=16'd0;
            end
        else begin
            cnt<=cnt+16'd1;
            baud_tick<=1'b0;
            end        
        end
    end



endmodule