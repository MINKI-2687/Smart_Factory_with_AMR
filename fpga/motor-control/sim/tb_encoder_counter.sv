`timescale 1ns / 1ps

module tb_encoder_counter();
    
    reg clk;
    reg rst;
    reg enc_pin;
    
    wire [31:0] count;

    encoder_counter u_encoder (
        .clk(clk),
        .rst(rst),
        .enc_pin(enc_pin),
        .count(count)
    );

    // 100MHz 클럭 생성
    always #5 clk = ~clk;

    initial begin
        // 초기화
        clk = 0;
        rst = 1;
        enc_pin = 1; // 적외선 센서 기본 상태 (막히지 않음 = High 가정)

        #100;
        rst = 0;
        #100;

        // ----------------------------------------------------
        // 엔코더 원판이 센서를 통과하는 상황 시뮬레이션
        // ----------------------------------------------------
        $display("[%0t] Test: Encoder Pulse Counting", $time);
        
        // 원판 구멍이 센서를 5번 지나는 상황 (Falling Edge 5번)
        repeat(5) begin
            enc_pin = 0; // 구멍에 막혀서 0이 됨 (Falling Edge 발생 -> 카운트 증가해야함)
            #1000;
            enc_pin = 1; // 다시 빛이 통과해서 1이 됨
            #1000;
        end
        
        #5000;
        $display("[%0t] Encoder Counter Simulation Complete!", $time);
        $finish;
    end
endmodule
