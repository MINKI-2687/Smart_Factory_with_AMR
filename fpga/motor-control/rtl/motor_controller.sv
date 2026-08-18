`timescale 1ns / 1ps

module pwm_generator (
    input clk,          // 100MHz (Basys3)
    input rst,
    input [7:0] duty,   // 0~255 (0% ~ 100% duty cycle)
    output reg pwm_out
);
    // 100MHz -> 20 분주 -> 5MHz, 5MHz/256 ≈ 19.5kHz (가청주파수 위로 두어 모터 소음 방지)
    localparam PRESCALER = 20;
    reg [4:0] prescaler;
    reg [7:0] counter;   
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            prescaler <= 0;
            counter <= 0;
            pwm_out <= 0;
        end else begin
            if (prescaler == PRESCALER - 1) begin
                prescaler <= 0;
                counter <= counter + 1; // 5MHz / 256 ≈ 19.5kHz의 PWM 주파수
            end else begin
                prescaler <= prescaler + 1;
            end
            
            // Duty Cycle 비교 (0이면 계속 0, 255면 거의 항상 1)
            if (counter < duty)
                pwm_out <= 1'b1;
            else
                pwm_out <= 1'b0;
        end
    end
endmodule

module motor_controller (
    input clk,
    input rst,
    
    // FSM에서 받아올 제어 신호
    input [7:0] duty_l, // 좌측 모터 속도 (0~255)
    input [7:0] duty_r, // 우측 모터 속도 (0~255)
    input dir_l,        // 좌측 모터 방향 (1: 직진, 0: 후진)
    input dir_r,        // 우측 모터 방향 (1: 직진, 0: 후진)
    input brake_l,      // 좌측 모터 급정거 (1: 정지, 0: 주행)
    input brake_r,      // 우측 모터 급정거 (1: 정지, 0: 주행)
    
    // L298N 모터 드라이버로 나가는 출력 핀
    output ena, // 좌측 모터 PWM
    output in1, // 좌측 모터 방향 1
    output in2, // 좌측 모터 방향 2
    
    output enb, // 우측 모터 PWM
    output in3, // 우측 모터 방향 1
    output in4  // 우측 모터 방향 2
);

    // 좌/우 PWM 파형 (브레이크 여부와 무관하게 duty 값 그대로 생성)
    wire pwm_out_l, pwm_out_r;

    // 좌측 모터 PWM 생성
    pwm_generator pwm_left (
        .clk(clk),
        .rst(rst),
        .duty(duty_l),
        .pwm_out(pwm_out_l)
    );

    // 우측 모터 PWM 생성
    pwm_generator pwm_right (
        .clk(clk),
        .rst(rst),
        .duty(duty_r),
        .pwm_out(pwm_out_r)
    );
    
    // L298N 방향 및 급정거(Brake) 제어 로직
    // brake가 1일 때: IN 양쪽 모두 1 + EN도 강제 HIGH로 켜야 실제 전자기적(단락) 급정거가 걸림.
    //   (EN=0이면 IN값과 무관하게 출력단이 하이임피던스가 되어 "브레이크"가 아니라 "코스트"가 됨)
    // brake가 0일 때: dir 값에 따라 정/역방향 회전, EN은 PWM 파형 그대로 출력
    assign in1 = brake_l ? 1'b1 : ~dir_l;
    assign in2 = brake_l ? 1'b1 : dir_l;
    assign ena = brake_l ? 1'b1 : pwm_out_l;
    
    assign in3 = brake_r ? 1'b1 : ~dir_r;
    assign in4 = brake_r ? 1'b1 : dir_r;
    assign enb = brake_r ? 1'b1 : pwm_out_r;

endmodule
