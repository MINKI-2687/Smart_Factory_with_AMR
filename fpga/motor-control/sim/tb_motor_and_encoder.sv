`timescale 1ns / 1ps

module tb_motor_and_encoder();

    // 공통 클럭 및 리셋
    reg clk;
    reg rst;

    // --- motor_controller 입력/출력 ---
    reg [7:0] duty_l, duty_r;
    reg dir_l, dir_r;
    reg brake_l, brake_r;
    
    wire ena, in1, in2;
    wire enb, in3, in4;

    // --- encoder_counter 입력/출력 ---
    reg enc_pin;
    wire [31:0] count;

    // 모터 제어 모듈 인스턴스화
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

    // 엔코더 카운터 모듈 인스턴스화
    encoder_counter u_encoder (
        .clk(clk),
        .rst(rst),
        .enc_pin(enc_pin),
        .count(count)
    );

    // 100MHz 클럭 생성 (주기 10ns)
    always #5 clk = ~clk;

    // 시뮬레이션 시나리오
    initial begin
        // 초기화
        clk = 0;
        rst = 1;
        duty_l = 0; duty_r = 0;
        dir_l = 1;  dir_r = 1;
        brake_l = 0; brake_r = 0;
        enc_pin = 1; // 엔코더 핀 기본 상태 High

        #100;
        rst = 0; // 리셋 해제
        #100;

        // ----------------------------------------------------
        // [테스트 1] 모터 속도(PWM) 및 방향, 브레이크 테스트
        // ----------------------------------------------------
        $display("[%0t] Test 1: Motor Forward, 50%%/25%% Duty", $time);
        dir_l = 1; dir_r = 1;       // 직진 (IN1=1, IN2=0)
        duty_l = 8'd128;            // 좌측: 약 50% 속도
        duty_r = 8'd64;             // 우측: 약 25% 속도
        
        // PWM 1주기가 256번 카운트하는 시간이므로 (1MHz 분주기 기준 약 256,000ns) 파형을 보기 위해 충분히 대기
        #500_000; 
        
        $display("[%0t] Test 2: Motor Backward, 100%% Duty", $time);
        dir_l = 0; dir_r = 0;       // 후진 (IN1=0, IN2=1)
        duty_l = 8'd255;            // 좌측: 100% 속도 (계속 High)
        duty_r = 8'd255;            // 우측: 100% 속도
        #500_000;

        $display("[%0t] Test 3: Fast Brake", $time);
        brake_l = 1; brake_r = 1;   // 급정거! (IN 핀들이 모두 0이 되어야 함)
        #100_000;
        brake_l = 0; brake_r = 0;

        // ----------------------------------------------------
        // [테스트 2] 엔코더 카운터 테스트 (Falling Edge)
        // ----------------------------------------------------
        $display("[%0t] Test 4: Encoder Pulse Counting", $time);
        
        // 임의로 바퀴가 돌면서 원판 구멍이 센서를 3번 지나가는 상황(Falling Edge 3번)을 시뮬레이션
        repeat(3) begin
            enc_pin = 0; // Falling Edge 발생 (빛 통과 상태 변화)
            #1000;
            enc_pin = 1; // 다시 High로 복구
            #1000;
        end
        
        // count 값이 3으로 증가했는지 확인하기 위한 여유 시간
        #5000;

        $display("[%0t] Simulation Complete!", $time);
        $finish;
    end

endmodule
