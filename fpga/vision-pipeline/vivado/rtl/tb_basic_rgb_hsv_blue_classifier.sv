`timescale 1ns / 1ps

module tb_basic_rgb_hsv_blue_classifier;
    logic aclk = 1'b0;
    logic aresetn = 1'b0;
    logic input_enable = 1'b0;
    logic product_enable = 1'b0;
    logic result_enable = 1'b0;
    logic [7:0] red;
    logic [7:0] green;
    logic [7:0] blue;
    logic blue_match;
    logic hue_in_range;
    logic saturation_in_range;
    logic value_in_range;
    int failures = 0;

    function automatic logic reference_match(
        input logic [7:0] ref_red,
        input logic [7:0] ref_green,
        input logic [7:0] ref_blue
    );
        int value_max;
        int value_min;
        int chroma;
        int hue_numerator;
        begin
            value_max = (ref_red >= ref_green && ref_red >= ref_blue) ? ref_red :
                        ((ref_green >= ref_blue) ? ref_green : ref_blue);
            value_min = (ref_red <= ref_green && ref_red <= ref_blue) ? ref_red :
                        ((ref_green <= ref_blue) ? ref_green : ref_blue);
            chroma = value_max - value_min;
            hue_numerator = 43 * ($signed({1'b0, ref_red}) -
                                  $signed({1'b0, ref_green}));
            reference_match =
                (ref_blue >= ref_red) && (ref_blue >= ref_green) &&
                (chroma != 0) &&
                (hue_numerator >= (-36 * chroma)) &&
                (hue_numerator <= (29 * chroma)) &&
                (value_max != 0) && ((255 * chroma) >= (70 * value_max)) &&
                (value_max >= 40);
        end
    endfunction

    basic_rgb_hsv_blue_classifier #(
        .HSV_HUE_MIN(135),
        .HSV_HUE_MAX(200),
        .HSV_SAT_MIN(70),
        .HSV_VALUE_MIN(40)
    ) dut (
        .aclk(aclk),
        .aresetn(aresetn),
        .input_enable(input_enable),
        .product_enable(product_enable),
        .result_enable(result_enable),
        .red(red),
        .green(green),
        .blue(blue),
        .blue_match(blue_match),
        .hue_in_range(hue_in_range),
        .saturation_in_range(saturation_in_range),
        .value_in_range(value_in_range)
    );

    always #5 aclk = ~aclk;

    task automatic run_pipeline(
        input logic [7:0] test_red,
        input logic [7:0] test_green,
        input logic [7:0] test_blue
    );
        begin
            @(negedge aclk);
            red = test_red;
            green = test_green;
            blue = test_blue;
            input_enable = 1'b1;
            @(posedge aclk);
            #1;
            input_enable = 1'b0;
            product_enable = 1'b1;
            @(posedge aclk);
            #1;
            product_enable = 1'b0;
            result_enable = 1'b1;
            @(posedge aclk);
            #1;
            result_enable = 1'b0;
        end
    endtask

    task automatic check_pixel(
        input logic [7:0] test_red,
        input logic [7:0] test_green,
        input logic [7:0] test_blue,
        input logic expected,
        input string label
    );
        begin
            run_pipeline(test_red, test_green, test_blue);
            if (blue_match !== expected) begin
                $display(
                    "FAIL %-24s RGB=(%0d,%0d,%0d) match=%0b H=%0b S=%0b V=%0b",
                    label, red, green, blue, blue_match,
                    hue_in_range, saturation_in_range, value_in_range);
                failures++;
            end else begin
                $display(
                    "PASS %-24s RGB=(%0d,%0d,%0d) match=%0b",
                    label, red, green, blue, blue_match);
            end
        end
    endtask

    initial begin
        int test_red;
        int test_green;
        int test_blue;

        red = '0;
        green = '0;
        blue = '0;
        repeat (2) @(posedge aclk);
        aresetn = 1'b1;

        check_pixel(8'd0,   8'd0,   8'd255, 1'b1, "pure blue");
        check_pixel(8'd10,  8'd15,  8'd55,  1'b1, "dark blue below old B100");
        check_pixel(8'd70,  8'd65,  8'd110, 1'b1, "soft blue below old margin");
        check_pixel(8'd40,  8'd65,  8'd110, 1'b1, "cool blue under lighting");
        check_pixel(8'd255, 8'd255, 8'd255, 1'b0, "white background");
        check_pixel(8'd90,  8'd90,  8'd90,  1'b0, "gray background");
        check_pixel(8'd15,  8'd15,  8'd25,  1'b0, "too dark");
        check_pixel(8'd0,   8'd255, 8'd0,   1'b0, "green");
        check_pixel(8'd255, 8'd0,   8'd0,   1'b0, "red");
        check_pixel(8'd0,   8'd255, 8'd255, 1'b0, "cyan outside hue");
        check_pixel(8'd100, 8'd20,  8'd120, 1'b0, "purple outside hue");

        for (test_red = 0; test_red <= 255; test_red += 17) begin
            for (test_green = 0; test_green <= 255; test_green += 17) begin
                for (test_blue = 0; test_blue <= 255; test_blue += 17) begin
                    run_pipeline(test_red[7:0], test_green[7:0],
                                 test_blue[7:0]);
                    if (blue_match !== reference_match(red, green, blue)) begin
                        $display(
                            "FAIL reference mismatch RGB=(%0d,%0d,%0d) match=%0b ref=%0b",
                            red, green, blue, blue_match,
                            reference_match(red, green, blue));
                        failures++;
                    end
                end
            end
        end

        if (failures != 0) begin
            $fatal(1, "HSV classifier failures=%0d", failures);
        end
        $display("HSV_CLASSIFIER_TEST_PASS");
        $finish;
    end
endmodule
