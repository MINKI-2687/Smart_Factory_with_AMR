`timescale 1ns / 1ps

module encoder_tx (
    input clk,
    input rst,
    
    // Encoder 카운트 입력 (16-bit Signed)
    input signed [15:0] count_l,
    input signed [15:0] count_r,
    
    // UART TX Interface (uart_tx 모듈로 송신)
    output reg [7:0] tx_data,
    output reg tx_start,
    input tx_busy
);

    // 50ms 타이머 파라미터 (100MHz 기준)
    // 100,000,000 * 0.05 = 5,000,000 카운트
    localparam TIMER_MAX = 32'd4_999_999;
    
    // 송신 패킷 길이
    localparam PACKET_LEN = 5'd16;
    
    // FSM States
    localparam IDLE = 2'd0;
    localparam PREPARE = 2'd1;
    localparam SEND = 2'd2;
    localparam WAIT_TX = 2'd3;
    
    reg [1:0] state;
    reg [31:0] timer;
    reg [4:0] byte_cnt;
    reg [7:0] buffer [0:15];
    
    // 4비트를 16진수 ASCII 문자(8비트)로 변환하는 함수 (나눗셈 전혀 없음!)
    function [7:0] nibble2hex;
        input [3:0] nibble;
        begin
            if (nibble < 10)
                nibble2hex = nibble + 8'h30; // '0' ~ '9'
            else
                nibble2hex = nibble - 10 + 8'h41; // 'A' ~ 'F'
        end
    endfunction
    
    // 부호 및 절댓값 분리용 임시 변수
    wire [15:0] abs_l = (count_l[15]) ? -count_l : count_l;
    wire [15:0] abs_r = (count_r[15]) ? -count_r : count_r;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            timer <= 0;
            tx_start <= 0;
            byte_cnt <= 0;
            tx_data <= 8'd0;
        end else begin
            // 50ms 타이머 동작
            if (timer < TIMER_MAX) begin
                timer <= timer + 1;
            end else if (state == IDLE) begin
                // 타이머가 다 차고 IDLE 상태일 때 송신 시작 트리거
                timer <= 0;
                state <= PREPARE;
            end
            
            case (state)
                IDLE: begin
                    tx_start <= 0;
                end
                
                PREPARE: begin
                    // 패킷 기본 골격 구성
                    buffer[0] <= "P";
                    buffer[1] <= " ";
                    buffer[8] <= " ";
                    buffer[15] <= "\n";
                    
                    // 좌측 카운트 부호 
                    if (count_l < 0) buffer[2] <= "-";
                    else             buffer[2] <= "+";
                    
                    // 우측 카운트 부호
                    if (count_r < 0) buffer[9] <= "-";
                    else             buffer[9] <= "+";
                    
                    // 좌측 카운트 (16진수 4자리 + 앞자리 0 패딩)
                    buffer[3] <= "0";
                    buffer[4] <= nibble2hex(abs_l[15:12]);
                    buffer[5] <= nibble2hex(abs_l[11:8]);
                    buffer[6] <= nibble2hex(abs_l[7:4]);
                    buffer[7] <= nibble2hex(abs_l[3:0]);
                    
                    // 우측 카운트 (16진수 4자리 + 앞자리 0 패딩)
                    buffer[10] <= "0";
                    buffer[11] <= nibble2hex(abs_r[15:12]);
                    buffer[12] <= nibble2hex(abs_r[11:8]);
                    buffer[13] <= nibble2hex(abs_r[7:4]);
                    buffer[14] <= nibble2hex(abs_r[3:0]);
                    
                    byte_cnt <= 0;
                    state <= SEND;
                end
                
                SEND: begin
                    if (!tx_busy) begin // UART TX가 바쁘지 않을 때만 데이터 전송
                        tx_data <= buffer[byte_cnt];
                        tx_start <= 1;
                        state <= WAIT_TX;
                    end
                end
                
                WAIT_TX: begin
                    tx_start <= 0; // 1클럭 펄스로 start 신호 내림
                    if (tx_busy) begin
                        // UART가 데이터를 받아 바빠진 것을 확인
                        if (byte_cnt == PACKET_LEN - 1) begin
                            state <= IDLE; // 16바이트 모두 전송 완료
                        end else begin
                            byte_cnt <= byte_cnt + 1;
                            state <= SEND; // 다음 바이트 전송 준비
                        end
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule
