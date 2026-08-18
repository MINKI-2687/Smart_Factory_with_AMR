`timescale 1ns / 1ps

module basic_sr04_guard_wrapper (
    input  wire        clk,
    input  wire        resetn,
    input  wire        echo,
    input  wire        run_enable,
    output wire        trig,
    output wire        car_present,
    output wire        run_led,
    output wire [31:0] status_word
);
    reg run_enable_meta;
    reg run_enable_sync;
    wire [31:0] sr04_status_word;

    always @(posedge clk) begin
        if (!resetn) begin
            run_enable_meta <= 1'b0;
            run_enable_sync <= 1'b0;
        end else begin
            run_enable_meta <= run_enable;
            run_enable_sync <= run_enable_meta;
        end
    end

    assign run_led = run_enable_sync;
    assign status_word = {
        sr04_status_word[31:20],
        run_enable_sync,
        sr04_status_word[18:0]
    };

    basic_sr04_guard #(
        .CLK_HZ(50_000_000),
        .TRIGGER_PERIOD_US(60_000),
        .TRIGGER_PULSE_US(10),
        .ECHO_TIMEOUT_US(30_000),
        .PRESENT_MAX_US(1_160),
        .ABSENT_MIN_US(1_450),
        .FILTER_SAMPLES(4)
    ) basic_sr04_guard_i (
        .clk(clk),
        .resetn(resetn),
        .echo(echo),
        .trig(trig),
        .car_present(car_present),
        .status_word(sr04_status_word)
    );
endmodule
