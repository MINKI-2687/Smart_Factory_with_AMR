`timescale 1ns / 1ps

module tb_rc_car_top();

    reg clk;
    reg rst;
    
    reg uart_rx;
    wire uart_tx;
    
    reg enc_pin_l;
    reg enc_pin_r;
    
    wire ena, in1, in2;
    wire enb, in3, in4;

    rc_car_top uut (
        .clk(clk),
        .rst(rst),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .enc_pin_l(enc_pin_l),
        .enc_pin_r(enc_pin_r),
        .ena(ena),
        .in1(in1),
        .in2(in2),
        .enb(enb),
        .in3(in3),
        .in4(in4)
    );

    // 100MHz 클럭 생성
    always #5 clk = ~clk; 

    // UART 통신 속도 9600 baud (1비트당 약 104,167ns)
    localparam BIT_PERIOD = 104167; 
    
    // 1바이트를 UART RX로 쏘는 Task
    task send_char(input [7:0] c);
        integer i;
        begin
            uart_rx = 0; // Start bit
            #(BIT_PERIOD);
            for (i=0; i<8; i=i+1) begin
                uart_rx = c[i];
                #(BIT_PERIOD);
            end
            uart_rx = 1; // Stop bit
            #(BIT_PERIOD);
        end
    endtask

    // 12바이트 "M +128 -064\n" 전송 (좌측 50% 전진, 우측 25% 후진)
    task send_command_1;
        begin
            send_char("M"); send_char(" ");
            send_char("+"); send_char("1"); send_char("2"); send_char("8");
            send_char(" ");
            send_char("-"); send_char("0"); send_char("6"); send_char("4");
            send_char("\n");
        end
    endtask
    
    // 12바이트 "M +000 +000\n" 전송 (급정거)
    task send_brake;
        begin
            send_char("M"); send_char(" ");
            send_char("+"); send_char("0"); send_char("0"); send_char("0");
            send_char(" ");
            send_char("+"); send_char("0"); send_char("0"); send_char("0");
            send_char("\n");
        end
    endtask

    // 12바이트 "M +999 -050\n" 전송 (255를 초과하는 비정상 패킷)
    task send_invalid_command;
        begin
            send_char("M"); send_char(" ");
            send_char("+"); send_char("9"); send_char("9"); send_char("9");
            send_char(" ");
            send_char("-"); send_char("0"); send_char("5"); send_char("0");
            send_char("\n");
        end
    endtask

    initial begin
        // 초기화
        clk = 0;
        rst = 1;
        uart_rx = 1;
        enc_pin_l = 1;
        enc_pin_r = 1;

        #100;
        rst = 0;
        #1000;
        
        // ----------------------------------------------------
        // [테스트 1] 라즈베리파이 명령 수신 테스트 좌측 50% 전진, 우측 25% 후진
        // ----------------------------------------------------
        $display("[%0t] Test 1: Sending Motor Command M +128 -064", $time);
        send_command_1(); 
        
        // 명령어 전송 완료(약 12.5ms 소요됨) 후 모터 제어 신호가 제대로 나오는지 대기
        #5_000_000; // 5ms 관찰
        
        // ----------------------------------------------------
        // [테스트 2] 엔코더 동작 테스트 (바퀴 회전 시뮬레이션)
        // ----------------------------------------------------
        $display("[%0t] Test 2: Simulating Encoder Pulses", $time);
        repeat(5) begin
            enc_pin_l = 0; enc_pin_r = 0; // Falling edge
            #1000;
            enc_pin_l = 1; enc_pin_r = 1; // 원상복구
            #1000;
        end
        // 카운터가 증가(좌측) 및 감소(우측)하는지 확인
        
        // ----------------------------------------------------
        // [테스트 3] 송신부(TX) P 패킷 확인 대기
        // ----------------------------------------------------
        $display("[%0t] Test 3: Waiting for P packet TX (50ms timer)", $time);
        // encoder_tx가 50ms 마다 쏘므로, 50ms 이상 대기하면 TX 파형이 나옴
        #55_000_000; 

        // ----------------------------------------------------
        // [테스트 4] 전자 브레이크 명령 수신 테스트
        // ----------------------------------------------------
        $display("[%0t] Test 4: Sending Electronic Brake Command", $time);
        send_brake();
        #5_000_000;
        
        // ----------------------------------------------------
        // [테스트 5] 255 초과 예외 처리 (무시) 테스트
        // ----------------------------------------------------
        $display("[%0t] Test 5: Sending Invalid Command (PWM 999) - Should be Ignored!", $time);
        send_invalid_command();
        // 비정상 패킷이므로 모터 제어 신호가 바뀌지 않고 직전 상태(브레이크)를 유지해야 함
        #5_000_000;
        
        // ----------------------------------------------------
        // [테스트 6] 통신 단절 (Watchdog) 안전 브레이크 테스트
        // ----------------------------------------------------
        $display("[%0t] Test 6: Waiting for Watchdog Timeout (500ms)...", $time);
        // 아무 통신 없이 500ms 이상 방치. Watchdog이 터지며 브레이크가 걸리는지 확인
        // (주의: 시뮬레이션에서 500ms는 꽤 깁니다. Vivado에서 Run All 후 약 5~10초 소요)
        #550_000_000; 
        
        $display("[%0t] Simulation Finished!", $time);
        $finish;
    end
endmodule
