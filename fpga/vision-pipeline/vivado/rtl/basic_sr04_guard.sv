`timescale 1ns / 1ps

module basic_sr04_guard #(
    parameter int CLK_HZ = 50_000_000,
    parameter int TRIGGER_PERIOD_US = 60_000,
    parameter int TRIGGER_PULSE_US = 10,
    parameter int ECHO_TIMEOUT_US = 30_000,
    parameter int PRESENT_MAX_US = 1_160,
    parameter int ABSENT_MIN_US = 1_450,
    parameter int FILTER_SAMPLES = 4
) (
    input  logic        clk,
    input  logic        resetn,
    input  logic        echo,
    output logic        trig,
    output logic        car_present,
    output logic [31:0] status_word
);
    localparam int CYCLES_PER_US = CLK_HZ / 1_000_000;
    localparam int DIV_W = (CYCLES_PER_US <= 1) ? 1 : $clog2(CYCLES_PER_US);
    localparam int PERIOD_W = (TRIGGER_PERIOD_US <= 1) ? 1 : $clog2(TRIGGER_PERIOD_US + 1);
    localparam int TIMER_W = (ECHO_TIMEOUT_US <= 1) ? 1 : $clog2(ECHO_TIMEOUT_US + 1);
    localparam int FILTER_W = (FILTER_SAMPLES <= 1) ? 1 : $clog2(FILTER_SAMPLES + 1);

    localparam logic [1:0] WAIT_PERIOD = 2'd0;
    localparam logic [1:0] SEND_TRIGGER = 2'd1;
    localparam logic [1:0] WAIT_ECHO = 2'd2;
    localparam logic [1:0] MEASURE_ECHO = 2'd3;

    logic [DIV_W-1:0] us_divider;
    logic us_tick;

    (* ASYNC_REG = "TRUE" *) logic echo_meta;
    (* ASYNC_REG = "TRUE" *) logic echo_sync;
    logic echo_sync_d;
    logic echo_rise;
    logic echo_fall;

    logic [1:0] state;
    logic [PERIOD_W-1:0] period_us;
    logic [TIMER_W-1:0] state_us;
    logic [15:0] measured_echo_us;
    logic [15:0] last_echo_us;
    logic last_measurement_valid;
    logic sample_toggle;

    logic new_sample;
    logic sample_valid;
    logic [15:0] sample_echo_us;
    logic [FILTER_W-1:0] present_count;
    logic [FILTER_W-1:0] absent_count;

    assign echo_rise = echo_sync && !echo_sync_d;
    assign echo_fall = !echo_sync && echo_sync_d;

    always_comb begin
        status_word = 32'd0;
        status_word[15:0] = last_echo_us;
        status_word[16] = last_measurement_valid;
        status_word[17] = car_present;
        status_word[18] = sample_toggle;
        status_word[31:24] = 8'h53;
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            us_divider <= '0;
            us_tick <= 1'b0;
        end else if (us_divider == DIV_W'(CYCLES_PER_US - 1)) begin
            us_divider <= '0;
            us_tick <= 1'b1;
        end else begin
            us_divider <= us_divider + 1'b1;
            us_tick <= 1'b0;
        end
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            echo_meta <= 1'b0;
            echo_sync <= 1'b0;
            echo_sync_d <= 1'b0;
        end else begin
            echo_meta <= echo;
            echo_sync <= echo_meta;
            echo_sync_d <= echo_sync;
        end
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            state <= WAIT_PERIOD;
            period_us <= '0;
            state_us <= '0;
            measured_echo_us <= '0;
            last_echo_us <= 16'hffff;
            last_measurement_valid <= 1'b0;
            sample_toggle <= 1'b0;
            new_sample <= 1'b0;
            sample_valid <= 1'b0;
            sample_echo_us <= 16'hffff;
            trig <= 1'b0;
        end else begin
            new_sample <= 1'b0;

            case (state)
                WAIT_PERIOD: begin
                    trig <= 1'b0;
                    if (us_tick) begin
                        if (period_us == PERIOD_W'(TRIGGER_PERIOD_US - 1)) begin
                            period_us <= '0;
                            state_us <= '0;
                            trig <= 1'b1;
                            state <= SEND_TRIGGER;
                        end else begin
                            period_us <= period_us + 1'b1;
                        end
                    end
                end

                SEND_TRIGGER: begin
                    if (us_tick) begin
                        if (state_us == TIMER_W'(TRIGGER_PULSE_US - 1)) begin
                            trig <= 1'b0;
                            state_us <= '0;
                            state <= WAIT_ECHO;
                        end else begin
                            state_us <= state_us + 1'b1;
                        end
                    end
                end

                WAIT_ECHO: begin
                    if (echo_rise) begin
                        state_us <= '0;
                        measured_echo_us <= '0;
                        state <= MEASURE_ECHO;
                    end else if (us_tick) begin
                        if (state_us == TIMER_W'(ECHO_TIMEOUT_US - 1)) begin
                            last_echo_us <= 16'hffff;
                            last_measurement_valid <= 1'b0;
                            sample_echo_us <= 16'hffff;
                            sample_valid <= 1'b0;
                            sample_toggle <= ~sample_toggle;
                            new_sample <= 1'b1;
                            state_us <= '0;
                            period_us <= '0;
                            state <= WAIT_PERIOD;
                        end else begin
                            state_us <= state_us + 1'b1;
                        end
                    end
                end

                default: begin
                    if (echo_fall) begin
                        last_echo_us <= measured_echo_us;
                        last_measurement_valid <= 1'b1;
                        sample_echo_us <= measured_echo_us;
                        sample_valid <= 1'b1;
                        sample_toggle <= ~sample_toggle;
                        new_sample <= 1'b1;
                        state_us <= '0;
                        period_us <= '0;
                        state <= WAIT_PERIOD;
                    end else if (us_tick) begin
                        if (state_us == TIMER_W'(ECHO_TIMEOUT_US - 1)) begin
                            last_echo_us <= 16'hffff;
                            last_measurement_valid <= 1'b0;
                            sample_echo_us <= 16'hffff;
                            sample_valid <= 1'b0;
                            sample_toggle <= ~sample_toggle;
                            new_sample <= 1'b1;
                            state_us <= '0;
                            period_us <= '0;
                            state <= WAIT_PERIOD;
                        end else begin
                            state_us <= state_us + 1'b1;
                            if (measured_echo_us != 16'hfffe)
                                measured_echo_us <= measured_echo_us + 1'b1;
                        end
                    end
                end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            car_present <= 1'b0;
            present_count <= '0;
            absent_count <= '0;
        end else if (new_sample) begin
            if (!car_present) begin
                absent_count <= '0;
                if (sample_valid && (sample_echo_us <= PRESENT_MAX_US)) begin
                    if (present_count == FILTER_W'(FILTER_SAMPLES - 1)) begin
                        car_present <= 1'b1;
                        present_count <= '0;
                    end else begin
                        present_count <= present_count + 1'b1;
                    end
                end else begin
                    present_count <= '0;
                end
            end else begin
                present_count <= '0;
                if (!sample_valid || (sample_echo_us >= ABSENT_MIN_US)) begin
                    if (absent_count == FILTER_W'(FILTER_SAMPLES - 1)) begin
                        car_present <= 1'b0;
                        absent_count <= '0;
                    end else begin
                        absent_count <= absent_count + 1'b1;
                    end
                end else begin
                    absent_count <= '0;
                end
            end
        end
    end
endmodule
