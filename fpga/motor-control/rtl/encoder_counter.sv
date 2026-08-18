`timescale 1ns / 1ps

module encoder_counter (
    input clk,
    input rst,
    input enc_pin,            // 엔코더 센서 핀
    input dir,                // 모터 회전 방향 (1: 전진, 0: 후진)
    output reg signed [15:0] count // 16-bit Signed 누적 카운트 (-32768 ~ +32767)
);
    // 2-Stage Synchronizer
    reg enc_sync_1, enc_sync_2;
    reg enc_prev;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            enc_sync_1 <= 1'b0;
            enc_sync_2 <= 1'b0;
            enc_prev <= 1'b0;
            count <= 16'd0;
        end else begin
            // 비동기 신호 동기화
            enc_sync_1 <= enc_pin;
            enc_sync_2 <= enc_sync_1;
            enc_prev <= enc_sync_2;
            
            // Falling Edge 검출
            if (enc_prev == 1'b1 && enc_sync_2 == 1'b0) begin
                if (dir == 1'b1)
                    count <= count + 1; // 전진 시 카운트 증가
                else
                    count <= count - 1; // 후진 시 카운트 감소
            end
        end
    end
endmodule
