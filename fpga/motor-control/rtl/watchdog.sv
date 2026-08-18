`timescale 1ns / 1ps

module watchdog (
    input clk,
    input rst,
    
    // 정상 명령 수신 시 들어오는 펄스 (1클럭)
    input valid_cmd_tick,
    
    // 타임아웃 시 발생하는 안전 브레이크 신호
    output reg watchdog_brake
);

    // 500ms 타이머 파라미터 (100MHz 기준)
    // 100,000,000 * 0.5 = 50,000,000 카운트
    localparam TIMER_MAX = 32'd49_999_999;
    
    reg [31:0] timer;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            timer <= 0;
            watchdog_brake <= 0;
        end else begin
            if (valid_cmd_tick) begin
                // 정상 명령이 들어오면 타이머 리셋 (밥 주기)
                timer <= 0;
                watchdog_brake <= 0;
            end else begin
                if (timer < TIMER_MAX) begin
                    timer <= timer + 1;
                end else begin
                    // 500ms 동안 명령이 없으면 브레이크 발동
                    watchdog_brake <= 1;
                end
            end
        end
    end
endmodule
