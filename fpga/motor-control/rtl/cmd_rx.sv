`timescale 1ns / 1ps

module cmd_rx (
    input clk,
    input rst,
    
    // UART RX Interface
    input [7:0] rx_data,
    input rx_done,
    
    // Motor Controller Outputs
    output reg [7:0] duty_l,
    output reg [7:0] duty_r,
    output reg dir_l,
    output reg dir_r,
    output reg brake_l,
    output reg brake_r,
    
    // Watchdog Timer Reset Pulse
    output reg valid_cmd_tick
);

    // FSM States
    localparam IDLE = 2'd0;
    localparam RECV = 2'd1;
    localparam EXEC = 2'd2;
    
    reg [1:0] state;
    reg [3:0] byte_cnt;
    reg [3:0] target_len;
    reg [7:0] buffer [0:15]; // 최대 16바이트 버퍼
    
    // ASCII to Binary (000~999까지 표현 가능하도록 10비트로 반환).
    // 실제 duty_l/duty_r는 8비트(0~255)라서, 이 값을 그대로 쓰기 전에
    // 255 초과 여부를 EXEC 상태에서 검사해서 벗어나면 패킷 전체를 폐기한다.
    function [9:0] ascii2bin;
        input [7:0] b100, b10, b1;
        reg [9:0] v100, v10, v1;
        begin
            v100 = b100 - 8'h30;
            v10  = b10 - 8'h30;
            v1   = b1 - 8'h30;
            
            // *100과 *10 곱셈에서 DSP(무거운 곱셈기) 블록이 사용되어 1.3ns 에러가 발생함.
            // 이를 막기 위해 시프트(<<)와 덧셈(+) 만으로 100배와 10배를 구현하여 속도 극대화.
            // 100x = 64x + 32x + 4x  /  10x = 8x + 2x
            ascii2bin = (v100 << 6) + (v100 << 5) + (v100 << 2) + (v10 << 3) + (v10 << 1) + v1;
        end
    endfunction

    // EXEC 상태에서 쓰는 임시 변환값 (0~255 범위 검증용)
    // always 블록 내부에서 blocking(=) 연산을 쓰면 타이밍 에러(-32ns)가 발생하므로 밖에서 wire로 실시간 계산되도록 변경!
    wire [9:0] w_pwm_l_raw = ascii2bin(buffer[3], buffer[4], buffer[5]);
    wire [9:0] w_pwm_r_raw = ascii2bin(buffer[8], buffer[9], buffer[10]);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            byte_cnt <= 0;
            target_len <= 0;
            duty_l <= 0; duty_r <= 0;
            dir_l <= 1; dir_r <= 1;
            brake_l <= 0; brake_r <= 0;
            valid_cmd_tick <= 1'b0;
        end else begin
            valid_cmd_tick <= 1'b0; // 펄스 초기화 (기본 0)
            
            case (state)
                // 1. 대기 상태 (Header 판별)
                IDLE: begin
                    if (rx_done) begin
                        case (rx_data)
                            "M": begin
                                buffer[0] <= rx_data;
                                byte_cnt <= 1;
                                target_len <= 12; // M 명령어는 총 12바이트
                                state <= RECV;
                            end
                            // 추후 "S"(서보), "L"(LED) 등 추가 가능
                            default: begin
                                // 모르는 헤더는 무시
                                state <= IDLE;
                            end
                        endcase
                    end
                end
                
                // 2. 가변 길이 수신 상태
                RECV: begin
                    if (rx_done) begin
                        buffer[byte_cnt] <= rx_data;
                        if (byte_cnt == target_len - 1) begin
                            state <= EXEC;
                        end else begin
                            byte_cnt <= byte_cnt + 1;
                        end
                    end
                end
                
                // 3. 해독 및 실행 상태
                EXEC: begin
                    case (buffer[0])
                        "M": begin
                            // 형식 검사 (공백 위치, 줄바꿈 위치)
                            // \n(LF) 또는 \r(CR) 모두 허용하도록 수정하여 테라텀 엔터키 호환성 확보
                            if (buffer[1] == " " && buffer[6] == " " && (buffer[11] == "\n" || buffer[11] == "\r")) begin

                                // PWM 값이 0~255 범위를 벗어나면(256~999) 패킷 전체를 무시하고
                                // 이전 값을 그대로 유지한다 (조용히 wrap되는 것을 방지).
                                if (w_pwm_l_raw <= 10'd255 && w_pwm_r_raw <= 10'd255) begin

                                    // 정상 패킷이므로 펄스 발생 (Watchdog 타이머 초기화용)
                                    valid_cmd_tick <= 1'b1;

                                    // 좌측 방향 및 속도
                                    if (buffer[2] == "+") dir_l <= 1'b1;
                                    else if (buffer[2] == "-") dir_l <= 1'b0;
                                    duty_l <= w_pwm_l_raw[7:0];

                                    // 우측 방향 및 속도
                                    if (buffer[7] == "+") dir_r <= 1'b1;
                                    else if (buffer[7] == "-") dir_r <= 1'b0;
                                    duty_r <= w_pwm_r_raw[7:0];

                                    // 전자 브레이크 검출 ("+000" 또는 "-000")
                                    if (buffer[3] == "0" && buffer[4] == "0" && buffer[5] == "0")
                                        brake_l <= 1'b1;
                                    else
                                        brake_l <= 1'b0;

                                    if (buffer[8] == "0" && buffer[9] == "0" && buffer[10] == "0")
                                        brake_r <= 1'b1;
                                    else
                                        brake_r <= 1'b0;
                                end
                                // 255 초과면 무시 (이전 상태 유지, valid_cmd_tick도 안 올림)
                            end
                            // 형식이 다르면 무시 (이전 상태 유지)
                        end
                    endcase
                    
                    state <= IDLE;
                    byte_cnt <= 0;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule
