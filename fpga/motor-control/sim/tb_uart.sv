`timescale 1ns / 1ps

module tb_uart();

    // Inputs
    reg clk;
    reg rst;
    reg uart_rx;

    // Outputs
    wire uart_tx;
    
    // Internal Wires for Loopback (Connecting RX out to TX in)
    wire [7:0] w_data;
    wire w_done;
    wire w_busy;
    wire w_tx_done;

    // Instantiate the Unit Under Test (UUT)
    uart_top uut (
        .clk(clk),
        .rst(rst),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .rx_data(w_data),
        .rx_done(w_done),
        .tx_data(w_data),
        .tx_start(w_done),
        .tx_busy(w_busy),
        .tx_done(w_tx_done)
    );

    // 100MHz Clock generation (10ns period)
    always #5 clk = ~clk;

    // UART 통신 속도 (Baud Rate 9600)에 따른 1비트 주기 (ns)
    // 1초(1,000,000,000 ns) / 9600 = 104166.6 ns
    localparam BIT_PERIOD = 104167; 

    // Task: 시뮬레이션 환경에서 UART 데이터를 1바이트 전송하는 함수
    task send_byte;
        input [7:0] data;
        integer i;
        begin
            // Start bit (Low)
            uart_rx = 0;
            #(BIT_PERIOD); 

            // 8 Data bits (LSB First)
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = data[i];
                #(BIT_PERIOD);
            end

            // Stop bit (High)
            uart_rx = 1;
            #(BIT_PERIOD);
        end
    endtask

    initial begin
        // Initialize Inputs
        clk = 0;
        rst = 1;
        uart_rx = 1; // UART의 기본 상태는 High (Idle)

        // Reset the system
        #100;
        rst = 0;
        #1000;

        // Test 1: 문자 'A' (Hex: 41) 전송
        $display("[%0t] Sending 'A' (8'h41)...", $time);
        send_byte(8'h41);

        // Loopback으로 반환되는 시간을 기다림 (10비트 주기 + 여유)
        #(BIT_PERIOD * 12);

        // Test 2: 문자 'B' (Hex: 42) 전송
        $display("[%0t] Sending 'B' (8'h42)...", $time);
        send_byte(8'h42);

        // Loopback 대기
        #(BIT_PERIOD * 12);

        $display("[%0t] Simulation Complete!", $time);
        $finish;
    end

endmodule
