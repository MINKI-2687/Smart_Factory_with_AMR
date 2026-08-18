`timescale 1ns / 1ps

module tb_motor_controller();

    reg clk;
    reg rst;

    // FSM에서 주는 제어 신호들
    reg [7:0] duty_l, duty_r;
    reg dir_l, dir_r;
    reg brake_l, brake_r;
    
    // L298N으로 나가는 출력 신호들
    wire ena, in1, in2;
    wire enb, in3, in4;

    motor_controller u_motor (
        .clk(clk),
        .rst(rst),
        .duty_l(duty_l),
        .duty_r(duty_r),
        .dir_l(dir_l),
        .dir_r(dir_r),
        .brake_l(brake_l),
        .brake_r(brake_r),
        .ena(ena),
        .in1(in1),
        .in2(in2),
        .enb(enb),
        .in3(in3),
        .in4(in4)
    );

    // 100MHz 클럭 생성 (주기 10ns)
    always #5 clk = ~clk;

    initial begin
        // 초기화
        clk = 0;
        rst = 1;
        duty_l = 0; duty_r = 0;
        dir_l = 1;  dir_r = 1;
        brake_l = 0; brake_r = 0;

        #100;
        rst = 0; // 리셋 해제
        #100;

        // ----------------------------------------------------
        // [테스트 1] 직진, 속도 다르게
        // ----------------------------------------------------
        $display("[%0t] Test 1: Motor Forward, 50%%/25%% Duty", $time);
        dir_l = 1; dir_r = 1;
        duty_l = 8'd128; // 50%
        duty_r = 8'd64;  // 25%
        #500_000; 
        
        // ----------------------------------------------------
        // [테스트 2] 후진, 최고 속도
        // ----------------------------------------------------
        $display("[%0t] Test 2: Motor Backward, 100%% Duty", $time);
        dir_l = 0; dir_r = 0;
        duty_l = 8'd255;
        duty_r = 8'd255;
        #500_000;

        // ----------------------------------------------------
        // [테스트 3] 급정거
        // ----------------------------------------------------
        $display("[%0t] Test 3: Fast Brake", $time);
        brake_l = 1; brake_r = 1;
        #100_000;
        brake_l = 0; brake_r = 0;

        $display("[%0t] Motor Controller Simulation Complete!", $time);
        $finish;
    end
endmodule
