`timescale 1ns / 1ps

`define UD #1

module UART_top(
    input clk,
    input uart_rx,
    input rst_n,
    input tx_en,
    //input [7:0]tx_data,
    
    output rx_valid,
    output [7:0]rx_data,
    output     uart_tx
    //output [7:0]inform,
  
   );
    parameter div = 16'd321;
    parameter div_16x = 20;
    
    reg [7:0]tx_data;//可以取代input or output
   

    wire rx_busy;
    wire tx_busy;
    wire tx_done;   //测试用，以后当IO

    wire baud_tick1;
    wire baud_tick1_16x;
    
    rx uart_rx1(
        .clk(clk),
        .rst_n(rst_n),
        .tick_16x(baud1_tick_16x),     // 16x 波特率脉冲
        .rxd(uart_rx),          // 串行输入（异步）
        .rx_data(rx_data),      // 接收到的字节
        .rx_valid(rx_valid),     // 接收有效（1 拍脉冲）
        .rx_busy(rx_busy)       // 忙标志
    );
     tx uart_tx1(
        .clk(clk),
        .rst_n(rst_n),        // 异步复位，低有效
        .baud_tick(baud1_tick),    // 波特率脉冲（来自 baud_gen）
        .tx_start(tx_en),     // 启动发送（1 拍脉冲）
        .tx_data(tx_data),      // 待发送字节
        .txd(uart_tx),          // 串行输出线
        .tx_busy(tx_busy),      // 忙标志
        .tx_done(tx_done)       // 发送完成（1 拍脉冲）
   );

     baud_gen baud_gen(
        .clk(clk),       // 系统时钟
        .rst_n(rst_n),     // 异步复位，低有效
        .div(div),       // 分频除数 = Fclk / Baud
        .baud_tick(baud1_tick)  // 波特率脉冲（高有效，宽 1 个 clk）
   );
    baud_gen baud_gen1(
        .clk(clk),       // 系统时钟
        .rst_n(rst_n),     // 异步复位，低有效
        .div(div_16x),       // 分频除数 = Fclk / Baud
        .baud_tick(baud1_tick_16x)  // 波特率脉冲（高有效，宽 1 个 clk）
   );
   // assign inform = {4'd0, tx_done, tx_busy, rx_busy, rx_valid};
endmodule