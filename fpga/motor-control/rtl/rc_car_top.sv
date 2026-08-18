`timescale 1ns / 1ps

module rc_car_top (
    input clk,          // 100MHz Basys3 클럭
    input rst,          // 리셋 버튼
    input [15:0] sw,    // Basys3 슬라이드 스위치 (하드웨어 테스트용)
    
    // UART Interface (Raspberry Pi 연동)
    input uart_rx,      // 라즈베리파이의 TX 핀에 연결
    output uart_tx,     // 라즈베리파이의 RX 핀에 연결
    
    // Motor Encoder Inputs (적외선 센서 등)
    input enc_pin_l,    // 좌측 바퀴 엔코더 핀
    input enc_pin_r,    // 우측 바퀴 엔코더 핀
    
    // L298N Motor Driver Outputs (모터로 전달)
    output ena,         // 좌측 모터 PWM (Enable A)
    output in1,         // 좌측 모터 방향 1
    output in2,         // 좌측 모터 방향 2
    output enb,         // 우측 모터 PWM (Enable B)
    output in3,         // 우측 모터 방향 1
    output in4          // 우측 모터 방향 2
);

    // =========================================================================
    // 1. 내부 통신용 Wires 선언
    // =========================================================================
    
    // UART RX/TX Wires
    wire [7:0] w_rx_data;
    wire w_rx_done;
    wire [7:0] w_tx_data;
    wire w_tx_start;
    wire w_tx_busy;
    
    // FSM -> 모터 컨트롤러 제어 신호
    wire [7:0] w_duty_l, w_duty_r;
    wire w_dir_l, w_dir_r;
    wire w_rx_brake_l, w_rx_brake_r;
    wire w_valid_cmd_tick; // Watchdog 초기화용 펄스
    
    // 엔코더 카운트
    wire signed [15:0] w_count_l, w_count_r;
    
    // 안전장치 (Watchdog) 및 최종 브레이크 로직
    wire w_watchdog_brake;
    wire w_final_brake_l, w_final_brake_r;
    
    // ★ 안전 브레이크 통합 로직 (명령에 의한 브레이크 OR 타임아웃에 의한 브레이크)
    assign w_final_brake_l = w_rx_brake_l | w_watchdog_brake;
    assign w_final_brake_r = w_rx_brake_r | w_watchdog_brake;

    // =========================================================================
    // 2. 하위 모듈 인스턴스화 (레고 블록 조립)
    // =========================================================================

    // [1] UART 통신 모듈
    uart_top u_uart (
        .clk(clk),
        .rst(rst),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .rx_data(w_rx_data),
        .rx_done(w_rx_done),
        .tx_data(w_tx_data),
        .tx_start(w_tx_start),
        .tx_busy(w_tx_busy),
        .tx_done() // 미사용
    );

    // [2] 수신부 FSM (라즈베리파이 -> FPGA 명령 해독)
    cmd_rx u_cmd_rx (
        .clk(clk),
        .rst(rst),
        .rx_data(w_rx_data),
        .rx_done(w_rx_done),
        .duty_l(w_duty_l),
        .duty_r(w_duty_r),
        .dir_l(w_dir_l),
        .dir_r(w_dir_r),
        .brake_l(w_rx_brake_l),
        .brake_r(w_rx_brake_r),
        .valid_cmd_tick(w_valid_cmd_tick)
    );

    // [3] 안전장치 (Watchdog Timer - 500ms 단절 시 강제 제동)
    watchdog u_watchdog (
        .clk(clk),
        .rst(rst),
        .valid_cmd_tick(w_valid_cmd_tick),
        .watchdog_brake(w_watchdog_brake)
    );

    // [4] 좌측 엔코더 (방향에 따라 +- 카운트)
    encoder_counter u_enc_l (
        .clk(clk),
        .rst(rst),
        .enc_pin(enc_pin_l),
        .dir(w_dir_l),
        .count(w_count_l)
    );

    // [5] 우측 엔코더 (방향에 따라 +- 카운트)
    encoder_counter u_enc_r (
        .clk(clk),
        .rst(rst),
        .enc_pin(enc_pin_r),
        .dir(w_dir_r),
        .count(w_count_r)
    );

    // [6] 송신부 FSM (FPGA -> 라즈베리파이, 50ms마다 엔코더 값 전송)
    encoder_tx u_enc_tx (
        .clk(clk),
        .rst(rst),
        .count_l(w_count_l),
        .count_r(w_count_r),
        .tx_data(w_tx_data),
        .tx_start(w_tx_start),
        .tx_busy(w_tx_busy)
    );

    // [7] 최종 모터 제어 출력 (L298N 연동)
    
    // =========================================================================
    // ★ [추가] 수동 테스트 모드 다중화 (Multiplexing) 로직
    // sw[15] == 1 이면 수동 테스트 모드 활성화, 0 이면 라즈베리파이 자동 모드
    // =========================================================================
    // 싸구려 TT모터 + L298N 전압 강하 특성상 50%(128)에서는 마찰력을 못 이기고 멈출 수 있습니다.
    // 저속 모드를 70%(180)로 올려서 테스트합니다.
    wire [7:0] mux_duty_l = sw[15] ? (sw[14] ? 8'd255 : 8'd180) : w_duty_l;
    wire [7:0] mux_duty_r = sw[15] ? (sw[14] ? 8'd255 : 8'd180) : w_duty_r;
    wire mux_dir_l        = sw[15] ? sw[0] : w_dir_l;
    wire mux_dir_r        = sw[15] ? sw[1] : w_dir_r;
    wire mux_brake_l      = sw[15] ? sw[2] : w_final_brake_l;
    wire mux_brake_r      = sw[15] ? sw[3] : w_final_brake_r;

    motor_controller u_motor (
        .clk(clk),
        .rst(rst),
        .duty_l(mux_duty_l),
        .duty_r(mux_duty_r),
        .dir_l(mux_dir_l),
        .dir_r(mux_dir_r),
        .brake_l(mux_brake_l), 
        .brake_r(mux_brake_r),
        .ena(ena),
        .in1(in1),
        .in2(in2),
        .enb(enb),
        .in3(in3),
        .in4(in4)
    );

endmodule
