`timescale 1ns / 1ps

// Division-free HSV blue classifier for one RGB pixel.
//
// Hue is encoded on 0..255 for 0..360 degrees. Only the blue-max sector can
// match this detector, so value=B and chroma=B-min(R,G). Expanding the hue and
// saturation comparisons removes both division and subtraction from the timed
// decision path:
//   R>=G: 43*R <= (Hmax-171)*B + (214-Hmax)*G
//   G>R : 43*G <= (171-Hmin)*B + (Hmin-128)*R
// This is mathematically equivalent to the HSV test for a hue window containing
// 171. RGB is registered first so the 150 MHz video path stays pipelined.
(* keep_hierarchy = "yes" *)
module basic_rgb_hsv_blue_classifier #(
    parameter int HSV_HUE_MIN = 135,
    parameter int HSV_HUE_MAX = 200,
    parameter int HSV_SAT_MIN = 70,
    parameter int HSV_VALUE_MIN = 40
) (
    input  logic       aclk,
    input  logic       aresetn,
    input  logic       input_enable,
    input  logic       product_enable,
    input  logic       result_enable,
    input  logic [7:0] red,
    input  logic [7:0] green,
    input  logic [7:0] blue,
    output logic       blue_match,
    output logic       hue_in_range,
    output logic       saturation_in_range,
    output logic       value_in_range
);
    localparam int HSV_BLUE_CENTER = 171;
    localparam int HSV_SECTOR_SCALE = 43;
    function automatic integer constant_gcd(
        input integer first,
        input integer second
    );
        integer a;
        integer b;
        integer remainder;
        begin
            a = first;
            b = second;
            while (b != 0) begin
                remainder = a % b;
                a = b;
                b = remainder;
            end
            constant_gcd = a;
        end
    endfunction

    localparam logic [7:0] HUE_LOW_DISTANCE =
        HSV_BLUE_CENTER - HSV_HUE_MIN;
    localparam logic [7:0] HUE_HIGH_DISTANCE =
        HSV_HUE_MAX - HSV_BLUE_CENTER;
    localparam logic [7:0] HUE_LOW_RED_SCALE =
        HSV_SECTOR_SCALE - HUE_LOW_DISTANCE;
    localparam logic [7:0] HUE_HIGH_GREEN_SCALE =
        HSV_SECTOR_SCALE - HUE_HIGH_DISTANCE;
    localparam int SATURATION_GCD = constant_gcd(255, HSV_SAT_MIN);
    localparam logic [7:0] SATURATION_BLUE_SCALE =
        (255 - HSV_SAT_MIN) / SATURATION_GCD;
    localparam logic [7:0] SATURATION_MIN_SCALE =
        255 / SATURATION_GCD;

    logic [7:0] red_reg;
    logic [7:0] green_reg;
    logic [7:0] blue_reg;
    logic [15:0] hue_red_scaled_reg;
    logic [15:0] hue_green_scaled_reg;
    logic [15:0] hue_blue_high_scaled_reg;
    logic [15:0] hue_green_high_scaled_reg;
    logic [15:0] hue_blue_low_scaled_reg;
    logic [15:0] hue_red_low_scaled_reg;
    logic [15:0] saturation_blue_scaled_reg;
    logic [15:0] saturation_red_scaled_reg;
    logic [15:0] saturation_green_scaled_reg;
    logic        red_is_above_green_reg;
    logic        blue_is_max_reg;
    logic        chroma_is_nonzero_reg;
    logic        blue_is_nonzero_reg;
    logic        value_in_range_reg;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            red_reg <= '0;
            green_reg <= '0;
            blue_reg <= '0;
        end else if (input_enable) begin
            red_reg <= red;
            green_reg <= green;
            blue_reg <= blue;
        end
    end

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            hue_red_scaled_reg <= '0;
            hue_green_scaled_reg <= '0;
            hue_blue_high_scaled_reg <= '0;
            hue_green_high_scaled_reg <= '0;
            hue_blue_low_scaled_reg <= '0;
            hue_red_low_scaled_reg <= '0;
            saturation_blue_scaled_reg <= '0;
            saturation_red_scaled_reg <= '0;
            saturation_green_scaled_reg <= '0;
            red_is_above_green_reg <= 1'b0;
            blue_is_max_reg <= 1'b0;
            chroma_is_nonzero_reg <= 1'b0;
            blue_is_nonzero_reg <= 1'b0;
            value_in_range_reg <= 1'b0;
        end else if (product_enable) begin
            hue_red_scaled_reg <= red_reg * HSV_SECTOR_SCALE;
            hue_green_scaled_reg <= green_reg * HSV_SECTOR_SCALE;
            hue_blue_high_scaled_reg <= blue_reg * HUE_HIGH_DISTANCE;
            hue_green_high_scaled_reg <= green_reg * HUE_HIGH_GREEN_SCALE;
            hue_blue_low_scaled_reg <= blue_reg * HUE_LOW_DISTANCE;
            hue_red_low_scaled_reg <= red_reg * HUE_LOW_RED_SCALE;
            saturation_blue_scaled_reg <=
                blue_reg * SATURATION_BLUE_SCALE;
            saturation_red_scaled_reg <=
                red_reg * SATURATION_MIN_SCALE;
            saturation_green_scaled_reg <=
                green_reg * SATURATION_MIN_SCALE;
            red_is_above_green_reg <= red_reg >= green_reg;
            blue_is_max_reg <=
                (blue_reg >= red_reg) && (blue_reg >= green_reg);
            chroma_is_nonzero_reg <=
                (blue_reg != red_reg) || (blue_reg != green_reg);
            blue_is_nonzero_reg <= blue_reg != 8'd0;
            value_in_range_reg <= blue_reg >= HSV_VALUE_MIN;
        end
    end

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            hue_in_range <= 1'b0;
            saturation_in_range <= 1'b0;
            value_in_range <= 1'b0;
        end else if (result_enable) begin
            hue_in_range <= blue_is_max_reg && chroma_is_nonzero_reg &&
                (red_is_above_green_reg ?
                 (hue_red_scaled_reg <=
                  (hue_blue_high_scaled_reg + hue_green_high_scaled_reg)) :
                 (hue_green_scaled_reg <=
                  (hue_blue_low_scaled_reg + hue_red_low_scaled_reg)));
            saturation_in_range <= blue_is_max_reg && blue_is_nonzero_reg &&
                (saturation_blue_scaled_reg >=
                 (red_is_above_green_reg ?
                  saturation_green_scaled_reg : saturation_red_scaled_reg));
            value_in_range <= blue_is_max_reg && value_in_range_reg;
        end
    end

    assign blue_match =
        hue_in_range && saturation_in_range && value_in_range;

endmodule
