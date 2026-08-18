`timescale 1ns / 1ps

// Multi-blob blue-target detector and overlay for the buffered display stream.
// Horizontal blue runs are associated with up to MAX_OBJECTS bounding boxes.
module basic_axis_blue_tracker_1080 #(
    parameter int H_RES = 1920,
    parameter int V_RES = 1080,
    parameter int XW = $clog2(H_RES),
    parameter int YW = $clog2(V_RES),
    parameter int AW = 24,
    parameter int MAX_OBJECTS = 6,
    parameter int PICK_X0 = 40,
    parameter int PICK_Y0 = 40,
    parameter int PICK_X1 = 1879,
    parameter int PICK_Y1 = 1039,
    parameter int MIN_AREA = 1200,
    parameter int MIN_W = 16,
    parameter int MIN_H = 16,
    parameter int MIN_RUN = 4,
    parameter int H_GAP = 8,
    parameter int V_GAP = 4,
    parameter int RUN_FIFO_DEPTH = 16,
    parameter int EDGE_MARGIN = 4,
    parameter int MIN_FILL_SHIFT = 3,
    parameter int STABLE_FRAMES = 3,
    parameter int STABLE_TOL = 24,
    // HSV uses 0..255 hue scaling: blue is centered near 171 (240 degrees).
    parameter int HSV_HUE_MIN = 135,
    parameter int HSV_HUE_MAX = 200,
    parameter int HSV_SAT_MIN = 70,
    parameter int HSV_VALUE_MIN = 40,
    parameter int BOX_THICKNESS = 3,
    parameter bit OVERLAY_ENABLE = 1'b1,
    parameter bit PICK_BORDER_ENABLE = 1'b0,
    parameter bit OSD_ENABLE = 1'b1,
    parameter int OSD_X = 24,
    parameter int OSD_Y = 24
) (
    input logic aclk,
    input logic aresetn,
    input logic [23:0] s_axis_tdata,
    input logic s_axis_tvalid,
    output logic s_axis_tready,
    input logic s_axis_tuser,
    input logic s_axis_tlast,
    output logic [23:0] m_axis_tdata,
    output logic m_axis_tvalid,
    input logic m_axis_tready,
    output logic m_axis_tuser,
    output logic m_axis_tlast,
    input logic pick_lock,
    input logic [2:0] coord_select,
    output logic coord_valid,
    output logic [XW-1:0] coord_x,
    output logic [YW-1:0] coord_y,
    output logic [2:0] object_count,
    output logic frame_done,
    output logic [31:0] coord_word
);
    localparam int OSD_TEXT_CHARS = 13;
    localparam int OSD_CELL_W = 16;
    localparam int OSD_CELL_H = 16;
    localparam int OIW = (MAX_OBJECTS <= 1) ? 1 : $clog2(MAX_OBJECTS);
    localparam int OCW = (MAX_OBJECTS <= 1) ? 1 : $clog2(MAX_OBJECTS + 1);
    localparam int BW = (XW > YW) ? XW : YW;
    localparam int BCW = (BW <= 1) ? 1 : $clog2(BW);
    localparam int SW = (STABLE_FRAMES <= 1) ? 1 : $clog2(STABLE_FRAMES + 1);
    localparam int RFW = (RUN_FIFO_DEPTH <= 2) ? 1 : $clog2(RUN_FIFO_DEPTH);
    localparam int RFCW = $clog2(RUN_FIFO_DEPTH + 1);

    localparam logic [1:0] EVAL_LOAD = 2'd0;
    localparam logic [1:0] EVAL_MULTIPLY = 2'd1;
    localparam logic [1:0] EVAL_ANALYZE = 2'd2;
    localparam logic [1:0] PROC_IDLE = 2'd0;
    localparam logic [1:0] PROC_LOAD = 2'd1;
    localparam logic [1:0] PROC_SCAN = 2'd2;
    localparam logic [1:0] PROC_COMMIT = 2'd3;
    localparam logic [1:0] COMMIT_SELECT = 2'd0;
    localparam logic [1:0] COMMIT_DISTANCE = 2'd1;
    localparam logic [1:0] COMMIT_CALC = 2'd2;
    localparam logic [1:0] COMMIT_STORE = 2'd3;

    logic [XW-1:0] x_pos;
    logic [YW-1:0] y_pos;
    logic [XW-1:0] pixel_x;
    logic [YW-1:0] pixel_y;
    logic [7:0] pix_r, pix_g, pix_b;
    logic hsv_blue_match;
    logic hsv_hue_in_range;
    logic hsv_saturation_in_range;
    logic hsv_value_in_range;
    logic in_pick_area, blue_hit;
    logic on_pick_border, on_target_box, on_center_cross, on_osd_text;
    logic osd_meta_valid;
    logic [3:0] osd_meta_char;
    logic [2:0] osd_meta_col, osd_meta_row;

    logic pre_valid, pre_user, pre_last;
    logic [23:0] pre_data;
    logic [XW-1:0] pre_pixel_x;
    logic [YW-1:0] pre_pixel_y;
    logic pre_in_pick_area;
    logic pre_pick_border, pre_target_box, pre_center_cross;
    logic pre_osd_meta_valid;
    logic [3:0] pre_osd_char;
    logic [2:0] pre_osd_col, pre_osd_row;
    logic hsv_product_valid, hsv_product_user, hsv_product_last;
    logic [23:0] hsv_product_data;
    logic [XW-1:0] hsv_product_pixel_x;
    logic [YW-1:0] hsv_product_pixel_y;
    logic hsv_product_in_pick_area;
    logic hsv_product_pick_border;
    logic hsv_product_target_box;
    logic hsv_product_center_cross;
    logic hsv_product_osd_meta_valid;
    logic [3:0] hsv_product_osd_char;
    logic [2:0] hsv_product_osd_col, hsv_product_osd_row;
    logic hsv_result_valid, hsv_result_user, hsv_result_last;
    logic [23:0] hsv_result_data;
    logic [XW-1:0] hsv_result_pixel_x;
    logic [YW-1:0] hsv_result_pixel_y;
    logic hsv_result_in_pick_area;
    logic hsv_result_pick_border;
    logic hsv_result_target_box;
    logic hsv_result_center_cross;
    logic hsv_result_osd_meta_valid;
    logic [3:0] hsv_result_osd_char;
    logic [2:0] hsv_result_osd_col, hsv_result_osd_row;
    logic stage1_valid, stage1_user, stage1_last;
    logic [23:0] stage1_data;
    logic [XW-1:0] stage1_pixel_x;
    logic [YW-1:0] stage1_pixel_y;
    logic stage1_blue_hit;
    logic stage1_pick_border, stage1_target_box, stage1_center_cross;
    logic stage1_osd_meta_valid;
    logic [3:0] stage1_osd_char;
    logic [2:0] stage1_osd_col, stage1_osd_row;
    logic stage2_valid, stage2_user, stage2_last;
    logic [23:0] stage2_data;
    logic stage2_osd_text;
    logic stage3_valid, stage3_user, stage3_last;
    logic [23:0] stage3_data;
    logic pre_ready, hsv_product_ready, hsv_result_ready;
    logic classifier_input_enable;
    logic classifier_product_enable;
    logic classifier_result_enable;
    logic stage1_ready, stage2_ready, stage3_ready, stage1_fire;

    logic run_active;
    logic [XW-1:0] run_start;
    logic [XW:0] run_length;
    logic run_finalize;
    logic [XW-1:0] finalized_start, finalized_end;
    logic [YW-1:0] finalized_y;
    logic [XW:0] finalized_length;
    logic run_enqueue, run_dequeue;
    logic input_frame_hold, fifo_near_full, frame_end_pending;

    logic [XW-1:0] run_fifo_start [0:RUN_FIFO_DEPTH-1];
    logic [XW-1:0] run_fifo_end [0:RUN_FIFO_DEPTH-1];
    logic [YW-1:0] run_fifo_y [0:RUN_FIFO_DEPTH-1];
    logic [XW:0] run_fifo_length [0:RUN_FIFO_DEPTH-1];
    logic [RFW-1:0] run_fifo_write_ptr, run_fifo_read_ptr;
    logic [RFCW-1:0] run_fifo_count;

    logic [1:0] proc_state;
    logic [OIW-1:0] proc_scan_index;
    logic [XW-1:0] proc_start, proc_end;
    logic [YW-1:0] proc_y;
    logic [XW:0] proc_length;
    logic proc_primary_valid, proc_free_valid;
    logic [OIW-1:0] proc_primary_index, proc_free_index, proc_smallest_index;
    logic [MAX_OBJECTS-1:0] proc_match_mask;
    logic [XW-1:0] proc_merged_left, proc_merged_right;
    logic [YW-1:0] proc_merged_top, proc_merged_bottom;
    logic [AW-1:0] proc_merged_area, proc_smallest_area;
    logic scan_work_valid;
    logic [XW-1:0] scan_work_left, scan_work_right;
    logic [YW-1:0] scan_work_top, scan_work_bottom;
    logic [AW-1:0] scan_work_area;
    logic proc_slot_match;

    logic work_valid [0:MAX_OBJECTS-1];
    logic [XW-1:0] work_left [0:MAX_OBJECTS-1];
    logic [XW-1:0] work_right [0:MAX_OBJECTS-1];
    logic [YW-1:0] work_top [0:MAX_OBJECTS-1];
    logic [YW-1:0] work_bottom [0:MAX_OBJECTS-1];
    logic [AW-1:0] work_area [0:MAX_OBJECTS-1];

    logic snapshot_pending;
    logic eval_active;
    logic [1:0] eval_phase;
    logic [OIW-1:0] eval_index;
    logic eval_valid [0:MAX_OBJECTS-1];
    logic [XW-1:0] eval_left [0:MAX_OBJECTS-1];
    logic [XW-1:0] eval_right [0:MAX_OBJECTS-1];
    logic [YW-1:0] eval_top [0:MAX_OBJECTS-1];
    logic [YW-1:0] eval_bottom [0:MAX_OBJECTS-1];
    logic [AW-1:0] eval_area [0:MAX_OBJECTS-1];

    logic candidate_source_valid;
    logic [XW-1:0] candidate_left, candidate_right, candidate_center_x;
    logic [YW-1:0] candidate_top, candidate_bottom, candidate_center_y;
    logic [XW:0] candidate_width;
    logic [YW:0] candidate_height;
    logic [AW-1:0] candidate_area, candidate_bbox_area;
    logic candidate_shape_valid;

    logic pending_valid [0:MAX_OBJECTS-1];
    logic [XW-1:0] pending_left [0:MAX_OBJECTS-1];
    logic [XW-1:0] pending_right [0:MAX_OBJECTS-1];
    logic [YW-1:0] pending_top [0:MAX_OBJECTS-1];
    logic [YW-1:0] pending_bottom [0:MAX_OBJECTS-1];
    logic [XW-1:0] pending_center_x [0:MAX_OBJECTS-1];
    logic [YW-1:0] pending_center_y [0:MAX_OBJECTS-1];
    logic [OCW-1:0] pending_count;

    logic track_valid [0:MAX_OBJECTS-1];
    logic [XW-1:0] track_center_x [0:MAX_OBJECTS-1];
    logic [YW-1:0] track_center_y [0:MAX_OBJECTS-1];
    logic [SW-1:0] track_stable [0:MAX_OBJECTS-1];

    logic box_valid [0:MAX_OBJECTS-1];
    logic [XW-1:0] box_left [0:MAX_OBJECTS-1];
    logic [XW-1:0] box_right [0:MAX_OBJECTS-1];
    logic [YW-1:0] box_top [0:MAX_OBJECTS-1];
    logic [YW-1:0] box_bottom [0:MAX_OBJECTS-1];
    logic [XW-1:0] box_center_x [0:MAX_OBJECTS-1];
    logic [YW-1:0] box_center_y [0:MAX_OBJECTS-1];
    logic [SW-1:0] box_stable [0:MAX_OBJECTS-1];

    logic commit_active, commit_update_enabled;
    logic [1:0] commit_phase;
    logic [OIW-1:0] commit_index;
    logic [OCW-1:0] commit_count;
    logic commit_pending_valid_reg, commit_track_valid_reg;
    logic [XW-1:0] commit_pending_x_reg, commit_track_x_reg;
    logic [YW-1:0] commit_pending_y_reg, commit_track_y_reg;
    logic [SW-1:0] commit_track_stable_reg;
    logic commit_x_near_reg, commit_y_near_reg;
    logic commit_slot_near;
    logic [SW-1:0] commit_slot_stable, commit_calc_stable;
    logic commit_slot_confirmed, commit_calc_confirmed;

    logic [2:0] coord_select_meta, coord_select_sync;
    logic [SW-1:0] selected_stable;

    logic bcd_active;
    logic [BCW-1:0] bcd_count;
    logic [XW-1:0] bcd_x_bin, bcd_last_x;
    logic [YW-1:0] bcd_y_bin, bcd_last_y;
    logic [2:0] bcd_select_bin, bcd_last_select;
    logic bcd_sample_valid;
    logic [XW-1:0] bcd_sample_x;
    logic [YW-1:0] bcd_sample_y;
    logic [2:0] bcd_sample_select;
    logic [15:0] bcd_x_work, bcd_y_work;
    logic [3:0] coord_x_d3, coord_x_d2, coord_x_d1, coord_x_d0;
    logic [3:0] coord_y_d3, coord_y_d2, coord_y_d1, coord_y_d0;

    function automatic logic [15:0] bcd_shift(input logic [15:0] bcd, input logic bit_in);
        logic [15:0] adjusted;
        int digit;
        begin
            adjusted = bcd;
            for (digit = 0; digit < 4; digit++) begin
                if (adjusted[digit * 4 +: 4] >= 4'd5)
                    adjusted[digit * 4 +: 4] = adjusted[digit * 4 +: 4] + 4'd3;
            end
            bcd_shift = {adjusted[14:0], bit_in};
        end
    endfunction

    function automatic logic [4:0] glyph_row(input logic [3:0] glyph, input logic [2:0] row);
        begin
            glyph_row = 5'b00000;
            case (glyph)
                4'd0: case (row) 0: glyph_row=5'b01110; 1,2,3,4,5: glyph_row=5'b10001; 6: glyph_row=5'b01110; endcase
                4'd1: case (row) 0: glyph_row=5'b00100; 1: glyph_row=5'b01100; 2,3,4,5: glyph_row=5'b00100; 6: glyph_row=5'b01110; endcase
                4'd2: case (row) 0: glyph_row=5'b01110; 1: glyph_row=5'b10001; 2: glyph_row=5'b00001; 3: glyph_row=5'b00010; 4: glyph_row=5'b00100; 5: glyph_row=5'b01000; 6: glyph_row=5'b11111; endcase
                4'd3: case (row) 0: glyph_row=5'b11110; 1,2: glyph_row=5'b00001; 3: glyph_row=5'b01110; 4,5: glyph_row=5'b00001; 6: glyph_row=5'b11110; endcase
                4'd4: case (row) 0: glyph_row=5'b00010; 1: glyph_row=5'b00110; 2: glyph_row=5'b01010; 3: glyph_row=5'b10010; 4: glyph_row=5'b11111; 5,6: glyph_row=5'b00010; endcase
                4'd5: case (row) 0: glyph_row=5'b11111; 1,2: glyph_row=5'b10000; 3: glyph_row=5'b11110; 4,5: glyph_row=5'b00001; 6: glyph_row=5'b11110; endcase
                4'd6: case (row) 0: glyph_row=5'b01110; 1,2: glyph_row=5'b10000; 3: glyph_row=5'b11110; 4,5: glyph_row=5'b10001; 6: glyph_row=5'b01110; endcase
                4'd7: case (row) 0: glyph_row=5'b11111; 1: glyph_row=5'b00001; 2: glyph_row=5'b00010; 3: glyph_row=5'b00100; 4,5,6: glyph_row=5'b01000; endcase
                4'd8: case (row) 0,3,6: glyph_row=5'b01110; 1,2,4,5: glyph_row=5'b10001; endcase
                4'd9: case (row) 0: glyph_row=5'b01110; 1,2: glyph_row=5'b10001; 3: glyph_row=5'b01111; 4,5: glyph_row=5'b00001; 6: glyph_row=5'b01110; endcase
                4'd10: case (row) 0,6: glyph_row=5'b10001; 1,5: glyph_row=5'b01010; 2,3,4: glyph_row=5'b00100; endcase
                4'd11: case (row) 0: glyph_row=5'b10001; 1: glyph_row=5'b01010; 2,3,4,5,6: glyph_row=5'b00100; endcase
                4'd12: case (row) 1,2,4,5: glyph_row=5'b00100; endcase
                default: glyph_row=5'b00000;
            endcase
        end
    endfunction

    assign stage3_ready = !stage3_valid || m_axis_tready;
    assign stage2_ready = !stage2_valid || stage3_ready;
    assign stage1_ready = !stage1_valid || stage2_ready;
    assign hsv_result_ready = !hsv_result_valid || stage1_ready;
    assign hsv_product_ready = !hsv_product_valid || hsv_result_ready;
    assign pre_ready = !pre_valid || hsv_product_ready;
    assign stage1_fire = stage1_valid && stage2_ready;
    assign fifo_near_full = (run_fifo_count >= (RUN_FIFO_DEPTH - 2));
    assign s_axis_tready = pre_ready && !input_frame_hold && !fifo_near_full;
    assign classifier_input_enable = s_axis_tvalid && s_axis_tready;
    assign classifier_product_enable = pre_valid && hsv_product_ready;
    assign classifier_result_enable =
        hsv_product_valid && hsv_result_ready;
    assign m_axis_tvalid = stage3_valid;
    assign m_axis_tuser = stage3_user;
    assign m_axis_tlast = stage3_last;
    assign m_axis_tdata = stage3_data;

    assign pixel_x = x_pos;
    assign pixel_y = y_pos;

    // Digilent's Pcam pipeline and rgb2dvi use 24-bit R-B-G packing.
    assign pix_r = s_axis_tdata[23:16];
    assign pix_b = s_axis_tdata[15:8];
    assign pix_g = s_axis_tdata[7:0];

    basic_rgb_hsv_blue_classifier #(
        .HSV_HUE_MIN(HSV_HUE_MIN),
        .HSV_HUE_MAX(HSV_HUE_MAX),
        .HSV_SAT_MIN(HSV_SAT_MIN),
        .HSV_VALUE_MIN(HSV_VALUE_MIN)
    ) hsv_blue_classifier_i (
        .aclk(aclk),
        .aresetn(aresetn),
        .input_enable(classifier_input_enable),
        .product_enable(classifier_product_enable),
        .result_enable(classifier_result_enable),
        .red(pix_r),
        .green(pix_g),
        .blue(pix_b),
        .blue_match(hsv_blue_match),
        .hue_in_range(hsv_hue_in_range),
        .saturation_in_range(hsv_saturation_in_range),
        .value_in_range(hsv_value_in_range)
    );

    assign in_pick_area = (pixel_x >= XW'(PICK_X0)) && (pixel_x <= XW'(PICK_X1)) &&
                          (pixel_y >= YW'(PICK_Y0)) && (pixel_y <= YW'(PICK_Y1));
    assign blue_hit = hsv_result_in_pick_area && hsv_blue_match;

    assign on_pick_border = PICK_BORDER_ENABLE &&
                            ((((pixel_x == XW'(PICK_X0)) || (pixel_x == XW'(PICK_X1))) &&
                              (pixel_y >= YW'(PICK_Y0)) && (pixel_y <= YW'(PICK_Y1))) ||
                             (((pixel_y == YW'(PICK_Y0)) || (pixel_y == YW'(PICK_Y1))) &&
                              (pixel_x >= XW'(PICK_X0)) && (pixel_x <= XW'(PICK_X1))));

    always_comb begin : p_multi_overlay
        int overlay_index;
        on_target_box = 1'b0;
        on_center_cross = 1'b0;
        for (overlay_index = 0; overlay_index < MAX_OBJECTS; overlay_index++) begin
            if (box_valid[overlay_index]) begin
                if (((((pixel_y >= box_top[overlay_index]) &&
                       (pixel_y <= box_top[overlay_index] + YW'(BOX_THICKNESS - 1))) ||
                      ((pixel_y + YW'(BOX_THICKNESS - 1) >= box_bottom[overlay_index]) &&
                       (pixel_y <= box_bottom[overlay_index]))) &&
                     (pixel_x >= box_left[overlay_index]) &&
                     (pixel_x <= box_right[overlay_index])) ||
                    ((((pixel_x >= box_left[overlay_index]) &&
                       (pixel_x <= box_left[overlay_index] + XW'(BOX_THICKNESS - 1))) ||
                      ((pixel_x + XW'(BOX_THICKNESS - 1) >= box_right[overlay_index]) &&
                       (pixel_x <= box_right[overlay_index]))) &&
                     (pixel_y >= box_top[overlay_index]) &&
                     (pixel_y <= box_bottom[overlay_index])))
                    on_target_box = 1'b1;

                if (((pixel_x + XW'(6) >= box_center_x[overlay_index]) &&
                     (pixel_x <= box_center_x[overlay_index] + XW'(6)) &&
                     (pixel_y == box_center_y[overlay_index])) ||
                    ((pixel_y + YW'(6) >= box_center_y[overlay_index]) &&
                     (pixel_y <= box_center_y[overlay_index] + YW'(6)) &&
                     (pixel_x == box_center_x[overlay_index])))
                    on_center_cross = 1'b1;
            end
        end
    end

    always_comb begin : p_selected_coordinate
        coord_valid = 1'b0;
        coord_x = '0;
        coord_y = '0;
        selected_stable = '0;
        if (coord_select_sync < MAX_OBJECTS) begin
            coord_valid = box_valid[coord_select_sync];
            coord_x = box_center_x[coord_select_sync];
            coord_y = box_center_y[coord_select_sync];
            selected_stable = box_stable[coord_select_sync];
        end

        coord_word = '0;
        coord_word[10:0] = coord_x;
        coord_word[21:11] = coord_y;
        coord_word[22] = coord_valid;
        coord_word[25:23] = object_count;
        coord_word[28:26] = coord_select_sync;
        coord_word[31:29] = selected_stable;
    end

    always_comb begin : p_osd_meta
        int local_x, local_y;
        osd_meta_valid = 1'b0;
        osd_meta_char = '0;
        osd_meta_col = '0;
        osd_meta_row = '0;
        local_x = 0;
        local_y = 0;
        if (OSD_ENABLE && coord_valid && (pixel_x >= XW'(OSD_X)) &&
            (pixel_x < XW'(OSD_X + OSD_TEXT_CHARS * OSD_CELL_W)) &&
            (pixel_y >= YW'(OSD_Y)) && (pixel_y < YW'(OSD_Y + OSD_CELL_H))) begin
            local_x = int'(pixel_x) - OSD_X;
            local_y = int'(pixel_y) - OSD_Y;
            osd_meta_valid = 1'b1;
            osd_meta_char = 4'(local_x >> 4);
            osd_meta_col = 3'((local_x & 15) >> 1);
            osd_meta_row = 3'(local_y >> 1);
        end
    end

    always_comb begin : p_osd_glyph
        logic [3:0] glyph_code;
        logic [4:0] glyph_bits;
        on_osd_text = 1'b0;
        glyph_code = 4'd15;
        glyph_bits = 5'b00000;
        if (stage1_osd_meta_valid) begin
            case (stage1_osd_char)
                0: glyph_code=4'd10; 1: glyph_code=4'd12;
                2: glyph_code=coord_x_d3; 3: glyph_code=coord_x_d2; 4: glyph_code=coord_x_d1; 5: glyph_code=coord_x_d0;
                7: glyph_code=4'd11; 8: glyph_code=4'd12;
                9: glyph_code=coord_y_d3; 10: glyph_code=coord_y_d2; 11: glyph_code=coord_y_d1; 12: glyph_code=coord_y_d0;
                default: glyph_code=4'd15;
            endcase
            if ((stage1_osd_col < 5) && (stage1_osd_row < 7)) begin
                glyph_bits = glyph_row(glyph_code, stage1_osd_row);
                on_osd_text = glyph_bits[4 - stage1_osd_col];
            end
        end
    end

    // Close a horizontal run when blue ends or at end-of-line.
    always_comb begin : p_finalize_run
        run_finalize = 1'b0;
        finalized_start = run_start;
        finalized_end = run_start;
        finalized_y = stage1_pixel_y;
        finalized_length = run_length;

        if (stage1_fire && !stage1_user) begin
            if (run_active && (!stage1_blue_hit || stage1_last)) begin
                run_finalize = 1'b1;
                if (stage1_blue_hit) begin
                    finalized_end = stage1_pixel_x;
                    finalized_length = run_length + 1'b1;
                end else begin
                    finalized_end = run_start + XW'(run_length - 1'b1);
                end
            end else if (!run_active && stage1_blue_hit && stage1_last) begin
                run_finalize = 1'b1;
                finalized_start = stage1_pixel_x;
                finalized_end = stage1_pixel_x;
                finalized_length = {{XW{1'b0}}, 1'b1};
            end
        end
    end

    assign run_enqueue = run_finalize && (finalized_length >= MIN_RUN) &&
                         (run_fifo_count < RUN_FIFO_DEPTH);
    assign run_dequeue = (proc_state == PROC_IDLE) && (run_fifo_count != 0);

    // One slot is examined per clock to keep the 150 MHz accumulator path short.
    always_comb begin : p_proc_slot_match
        proc_slot_match = scan_work_valid &&
            ({1'b0, proc_y} <= ({1'b0, scan_work_bottom} + (V_GAP + 1))) &&
            ({1'b0, proc_start} <= ({1'b0, scan_work_right} + H_GAP)) &&
            (({1'b0, proc_end} + H_GAP) >= {1'b0, scan_work_left});
    end

    assign candidate_shape_valid = candidate_source_valid &&
        (candidate_area >= AW'(MIN_AREA)) &&
        (candidate_width >= MIN_W) &&
        (candidate_height >= MIN_H) &&
        (candidate_area >= (candidate_bbox_area >> MIN_FILL_SHIFT)) &&
        (candidate_left > XW'(EDGE_MARGIN)) &&
        (candidate_top > YW'(EDGE_MARGIN)) &&
        (candidate_right < XW'(H_RES - 1 - EDGE_MARGIN)) &&
        (candidate_bottom < YW'(V_RES - 1 - EDGE_MARGIN));

    // Stability is evaluated one slot at a time, then registered before commit.
    always_comb begin : p_commit_slot
        commit_slot_near = commit_pending_valid_reg &&
            commit_track_valid_reg && commit_x_near_reg && commit_y_near_reg;

        if (!commit_pending_valid_reg)
            commit_slot_stable = '0;
        else if (commit_slot_near) begin
            if (commit_track_stable_reg < SW'(STABLE_FRAMES))
                commit_slot_stable = commit_track_stable_reg + SW'(1);
            else
                commit_slot_stable = commit_track_stable_reg;
        end else
            commit_slot_stable = SW'(1);

        commit_slot_confirmed = commit_pending_valid_reg &&
            ((STABLE_FRAMES <= 1) ||
             (commit_slot_near &&
              (commit_slot_stable >= SW'(STABLE_FRAMES))));
    end

    always_ff @(posedge aclk) begin : p_stream
        if (!aresetn) begin
            pre_valid <= 1'b0;
            pre_user <= 1'b0;
            pre_last <= 1'b0;
            pre_data <= '0;
            pre_pixel_x <= '0;
            pre_pixel_y <= '0;
            pre_in_pick_area <= 1'b0;
            pre_pick_border <= 1'b0;
            pre_target_box <= 1'b0;
            pre_center_cross <= 1'b0;
            pre_osd_meta_valid <= 1'b0;
            pre_osd_char <= '0;
            pre_osd_col <= '0;
            pre_osd_row <= '0;
            hsv_product_valid <= 1'b0;
            hsv_product_user <= 1'b0;
            hsv_product_last <= 1'b0;
            hsv_product_data <= '0;
            hsv_product_pixel_x <= '0;
            hsv_product_pixel_y <= '0;
            hsv_product_in_pick_area <= 1'b0;
            hsv_product_pick_border <= 1'b0;
            hsv_product_target_box <= 1'b0;
            hsv_product_center_cross <= 1'b0;
            hsv_product_osd_meta_valid <= 1'b0;
            hsv_product_osd_char <= '0;
            hsv_product_osd_col <= '0;
            hsv_product_osd_row <= '0;
            hsv_result_valid <= 1'b0;
            hsv_result_user <= 1'b0;
            hsv_result_last <= 1'b0;
            hsv_result_data <= '0;
            hsv_result_pixel_x <= '0;
            hsv_result_pixel_y <= '0;
            hsv_result_in_pick_area <= 1'b0;
            hsv_result_pick_border <= 1'b0;
            hsv_result_target_box <= 1'b0;
            hsv_result_center_cross <= 1'b0;
            hsv_result_osd_meta_valid <= 1'b0;
            hsv_result_osd_char <= '0;
            hsv_result_osd_col <= '0;
            hsv_result_osd_row <= '0;
            stage1_valid <= 1'b0;
            stage1_user <= 1'b0;
            stage1_last <= 1'b0;
            stage1_data <= '0;
            stage1_pixel_x <= '0;
            stage1_pixel_y <= '0;
            stage1_blue_hit <= 1'b0;
            stage1_pick_border <= 1'b0;
            stage1_target_box <= 1'b0;
            stage1_center_cross <= 1'b0;
            stage1_osd_meta_valid <= 1'b0;
            stage1_osd_char <= '0;
            stage1_osd_col <= '0;
            stage1_osd_row <= '0;
            stage2_valid <= 1'b0;
            stage2_user <= 1'b0;
            stage2_last <= 1'b0;
            stage2_data <= '0;
            stage2_osd_text <= 1'b0;
            stage3_valid <= 1'b0;
            stage3_user <= 1'b0;
            stage3_last <= 1'b0;
            stage3_data <= '0;
        end else begin
            if (stage3_ready) begin
                stage3_valid <= stage2_valid;
                if (stage2_valid) begin
                    stage3_user <= stage2_user;
                    stage3_last <= stage2_last;
                    stage3_data <= stage2_data;
                    if (OVERLAY_ENABLE && stage2_osd_text)
                        stage3_data <= 24'hFF_FF_FF;
                end
            end
            if (stage2_ready) begin
                stage2_valid <= stage1_valid;
                if (stage1_valid) begin
                    stage2_user <= stage1_user;
                    stage2_last <= stage1_last;
                    stage2_data <= stage1_data;
                    if (OVERLAY_ENABLE) begin
                        if (stage1_pick_border) stage2_data <= 24'hFF_FF_FF;
                        if (stage1_target_box) stage2_data <= 24'hFF_00_FF;
                        if (stage1_center_cross) stage2_data <= 24'hFF_00_00;
                    end
                    stage2_osd_text <= on_osd_text;
                end
            end
            if (stage1_ready) begin
                stage1_valid <= hsv_result_valid;
                if (hsv_result_valid) begin
                    stage1_user <= hsv_result_user;
                    stage1_last <= hsv_result_last;
                    stage1_data <= hsv_result_data;
                    stage1_pixel_x <= hsv_result_pixel_x;
                    stage1_pixel_y <= hsv_result_pixel_y;
                    stage1_blue_hit <= blue_hit;
                    stage1_pick_border <= hsv_result_pick_border;
                    stage1_target_box <= hsv_result_target_box;
                    stage1_center_cross <= hsv_result_center_cross;
                    stage1_osd_meta_valid <= hsv_result_osd_meta_valid;
                    stage1_osd_char <= hsv_result_osd_char;
                    stage1_osd_col <= hsv_result_osd_col;
                    stage1_osd_row <= hsv_result_osd_row;
                end
            end
            if (hsv_result_ready) begin
                hsv_result_valid <= classifier_result_enable;
                if (classifier_result_enable) begin
                    hsv_result_user <= hsv_product_user;
                    hsv_result_last <= hsv_product_last;
                    hsv_result_data <= hsv_product_data;
                    hsv_result_pixel_x <= hsv_product_pixel_x;
                    hsv_result_pixel_y <= hsv_product_pixel_y;
                    hsv_result_in_pick_area <= hsv_product_in_pick_area;
                    hsv_result_pick_border <= hsv_product_pick_border;
                    hsv_result_target_box <= hsv_product_target_box;
                    hsv_result_center_cross <= hsv_product_center_cross;
                    hsv_result_osd_meta_valid <=
                        hsv_product_osd_meta_valid;
                    hsv_result_osd_char <= hsv_product_osd_char;
                    hsv_result_osd_col <= hsv_product_osd_col;
                    hsv_result_osd_row <= hsv_product_osd_row;
                end
            end
            if (hsv_product_ready) begin
                hsv_product_valid <= classifier_product_enable;
                if (classifier_product_enable) begin
                    hsv_product_user <= pre_user;
                    hsv_product_last <= pre_last;
                    hsv_product_data <= pre_data;
                    hsv_product_pixel_x <= pre_pixel_x;
                    hsv_product_pixel_y <= pre_pixel_y;
                    hsv_product_in_pick_area <= pre_in_pick_area;
                    hsv_product_pick_border <= pre_pick_border;
                    hsv_product_target_box <= pre_target_box;
                    hsv_product_center_cross <= pre_center_cross;
                    hsv_product_osd_meta_valid <= pre_osd_meta_valid;
                    hsv_product_osd_char <= pre_osd_char;
                    hsv_product_osd_col <= pre_osd_col;
                    hsv_product_osd_row <= pre_osd_row;
                end
            end
            if (pre_ready) begin
                pre_valid <= classifier_input_enable;
                if (classifier_input_enable) begin
                    pre_user <= s_axis_tuser;
                    pre_last <= s_axis_tlast;
                    pre_data <= s_axis_tdata;
                    pre_pixel_x <= pixel_x;
                    pre_pixel_y <= pixel_y;
                    pre_in_pick_area <= in_pick_area;
                    pre_pick_border <= on_pick_border;
                    pre_target_box <= on_target_box;
                    pre_center_cross <= on_center_cross;
                    pre_osd_meta_valid <= osd_meta_valid;
                    pre_osd_char <= osd_meta_char;
                    pre_osd_col <= osd_meta_col;
                    pre_osd_row <= osd_meta_row;
                end
            end
        end
    end

    always_ff @(posedge aclk) begin : p_select_sync
        if (!aresetn) begin
            coord_select_meta <= '0;
            coord_select_sync <= '0;
        end else begin
            coord_select_meta <= coord_select;
            coord_select_sync <= coord_select_meta;
        end
    end

    always_ff @(posedge aclk) begin : p_detect
        int state_index;
        logic [15:0] bcd_next_x, bcd_next_y;
        if (!aresetn) begin
            x_pos <= '0;
            y_pos <= '0;
            run_active <= 1'b0;
            run_start <= '0;
            run_length <= '0;
            input_frame_hold <= 1'b0;
            frame_end_pending <= 1'b0;
            run_fifo_write_ptr <= '0;
            run_fifo_read_ptr <= '0;
            run_fifo_count <= '0;
            proc_state <= PROC_IDLE;
            proc_scan_index <= '0;
            proc_start <= '0;
            proc_end <= '0;
            proc_y <= '0;
            proc_length <= '0;
            proc_primary_valid <= 1'b0;
            proc_primary_index <= '0;
            proc_free_valid <= 1'b0;
            proc_free_index <= '0;
            proc_smallest_index <= '0;
            proc_match_mask <= '0;
            proc_merged_left <= '0;
            proc_merged_right <= '0;
            proc_merged_top <= '0;
            proc_merged_bottom <= '0;
            proc_merged_area <= '0;
            proc_smallest_area <= {AW{1'b1}};
            scan_work_valid <= 1'b0;
            scan_work_left <= '0;
            scan_work_right <= '0;
            scan_work_top <= '0;
            scan_work_bottom <= '0;
            scan_work_area <= '0;
            snapshot_pending <= 1'b0;
            eval_active <= 1'b0;
            eval_phase <= EVAL_LOAD;
            eval_index <= '0;
            candidate_source_valid <= 1'b0;
            candidate_left <= '0;
            candidate_right <= '0;
            candidate_top <= '0;
            candidate_bottom <= '0;
            candidate_center_x <= '0;
            candidate_center_y <= '0;
            candidate_width <= '0;
            candidate_height <= '0;
            candidate_area <= '0;
            candidate_bbox_area <= '0;
            pending_count <= '0;
            commit_active <= 1'b0;
            commit_phase <= COMMIT_SELECT;
            commit_update_enabled <= 1'b0;
            commit_index <= '0;
            commit_count <= '0;
            commit_pending_valid_reg <= 1'b0;
            commit_track_valid_reg <= 1'b0;
            commit_pending_x_reg <= '0;
            commit_track_x_reg <= '0;
            commit_pending_y_reg <= '0;
            commit_track_y_reg <= '0;
            commit_track_stable_reg <= '0;
            commit_x_near_reg <= 1'b0;
            commit_y_near_reg <= 1'b0;
            commit_calc_stable <= '0;
            commit_calc_confirmed <= 1'b0;
            object_count <= '0;
            frame_done <= 1'b0;
            bcd_active <= 1'b0;
            bcd_count <= '0;
            bcd_x_bin <= '0;
            bcd_y_bin <= '0;
            bcd_last_x <= '0;
            bcd_last_y <= '0;
            bcd_select_bin <= '0;
            bcd_last_select <= '0;
            bcd_sample_valid <= 1'b0;
            bcd_sample_x <= '0;
            bcd_sample_y <= '0;
            bcd_sample_select <= '0;
            bcd_x_work <= '0;
            bcd_y_work <= '0;
            coord_x_d3 <= '0;
            coord_x_d2 <= '0;
            coord_x_d1 <= '0;
            coord_x_d0 <= '0;
            coord_y_d3 <= '0;
            coord_y_d2 <= '0;
            coord_y_d1 <= '0;
            coord_y_d0 <= '0;
            for (state_index = 0; state_index < MAX_OBJECTS; state_index++) begin
                work_valid[state_index] <= 1'b0;
                work_left[state_index] <= '0;
                work_right[state_index] <= '0;
                work_top[state_index] <= '0;
                work_bottom[state_index] <= '0;
                work_area[state_index] <= '0;
                eval_valid[state_index] <= 1'b0;
                eval_left[state_index] <= '0;
                eval_right[state_index] <= '0;
                eval_top[state_index] <= '0;
                eval_bottom[state_index] <= '0;
                eval_area[state_index] <= '0;
                pending_valid[state_index] <= 1'b0;
                pending_left[state_index] <= '0;
                pending_right[state_index] <= '0;
                pending_top[state_index] <= '0;
                pending_bottom[state_index] <= '0;
                pending_center_x[state_index] <= '0;
                pending_center_y[state_index] <= '0;
                track_valid[state_index] <= 1'b0;
                track_center_x[state_index] <= '0;
                track_center_y[state_index] <= '0;
                track_stable[state_index] <= '0;
                box_valid[state_index] <= 1'b0;
                box_left[state_index] <= '0;
                box_right[state_index] <= '0;
                box_top[state_index] <= '0;
                box_bottom[state_index] <= '0;
                box_center_x[state_index] <= '0;
                box_center_y[state_index] <= '0;
                box_stable[state_index] <= '0;
            end
        end else begin
            frame_done <= 1'b0;
            bcd_sample_valid <= coord_valid;
            bcd_sample_x <= coord_x;
            bcd_sample_y <= coord_y;
            bcd_sample_select <= coord_select_sync;

            if (s_axis_tvalid && s_axis_tready) begin
                if (s_axis_tlast && (pixel_y == YW'(V_RES - 1)))
                    input_frame_hold <= 1'b1;
                if (s_axis_tuser) begin
                    x_pos <= s_axis_tlast ? '0 : XW'(1);
                    y_pos <= s_axis_tlast ? YW'(1) : '0;
                end else if (s_axis_tlast) begin
                    x_pos <= '0;
                    y_pos <= (pixel_y == YW'(V_RES - 1)) ? '0 : pixel_y + YW'(1);
                end else begin
                    x_pos <= pixel_x + XW'(1);
                    y_pos <= pixel_y;
                end
            end

            if (stage1_fire) begin
                if (stage1_user) begin
                    if (stage1_blue_hit && !stage1_last) begin
                        run_active <= 1'b1;
                        run_start <= stage1_pixel_x;
                        run_length <= {{XW{1'b0}}, 1'b1};
                    end else begin
                        run_active <= 1'b0;
                        run_start <= '0;
                        run_length <= '0;
                    end
                end else if (stage1_blue_hit) begin
                    if (run_active)
                        run_length <= run_length + 1'b1;
                    else begin
                        run_active <= 1'b1;
                        run_start <= stage1_pixel_x;
                        run_length <= {{XW{1'b0}}, 1'b1};
                    end
                end else begin
                    run_active <= 1'b0;
                    run_length <= '0;
                end

                if (stage1_last) begin
                    run_active <= 1'b0;
                    run_length <= '0;
                end
            end

            if (run_enqueue) begin
                run_fifo_start[run_fifo_write_ptr] <= finalized_start;
                run_fifo_end[run_fifo_write_ptr] <= finalized_end;
                run_fifo_y[run_fifo_write_ptr] <= finalized_y;
                run_fifo_length[run_fifo_write_ptr] <= finalized_length;
                run_fifo_write_ptr <= run_fifo_write_ptr + 1'b1;
            end
            if (run_dequeue)
                run_fifo_read_ptr <= run_fifo_read_ptr + 1'b1;

            case ({run_enqueue, run_dequeue})
                2'b10: run_fifo_count <= run_fifo_count + 1'b1;
                2'b01: run_fifo_count <= run_fifo_count - 1'b1;
                default: run_fifo_count <= run_fifo_count;
            endcase

            case (proc_state)
                PROC_IDLE: begin
                    if (run_dequeue) begin
                        proc_start <= run_fifo_start[run_fifo_read_ptr];
                        proc_end <= run_fifo_end[run_fifo_read_ptr];
                        proc_y <= run_fifo_y[run_fifo_read_ptr];
                        proc_length <= run_fifo_length[run_fifo_read_ptr];
                        proc_scan_index <= '0;
                        proc_primary_valid <= 1'b0;
                        proc_primary_index <= '0;
                        proc_free_valid <= 1'b0;
                        proc_free_index <= '0;
                        proc_smallest_index <= '0;
                        proc_match_mask <= '0;
                        proc_merged_left <= run_fifo_start[run_fifo_read_ptr];
                        proc_merged_right <= run_fifo_end[run_fifo_read_ptr];
                        proc_merged_top <= run_fifo_y[run_fifo_read_ptr];
                        proc_merged_bottom <= run_fifo_y[run_fifo_read_ptr];
                        proc_merged_area <= AW'(run_fifo_length[run_fifo_read_ptr]);
                        proc_smallest_area <= {AW{1'b1}};
                        proc_state <= PROC_LOAD;
                    end
                end
                PROC_LOAD: begin
                    scan_work_valid <= work_valid[proc_scan_index];
                    scan_work_left <= work_left[proc_scan_index];
                    scan_work_right <= work_right[proc_scan_index];
                    scan_work_top <= work_top[proc_scan_index];
                    scan_work_bottom <= work_bottom[proc_scan_index];
                    scan_work_area <= work_area[proc_scan_index];
                    proc_state <= PROC_SCAN;
                end
                PROC_SCAN: begin
                    if (!scan_work_valid && !proc_free_valid) begin
                        proc_free_valid <= 1'b1;
                        proc_free_index <= proc_scan_index;
                    end
                    if (scan_work_valid &&
                        (scan_work_area < proc_smallest_area)) begin
                        proc_smallest_area <= scan_work_area;
                        proc_smallest_index <= proc_scan_index;
                    end
                    if (proc_slot_match) begin
                        proc_match_mask[proc_scan_index] <= 1'b1;
                        if (!proc_primary_valid) begin
                            proc_primary_valid <= 1'b1;
                            proc_primary_index <= proc_scan_index;
                        end
                        if (scan_work_left < proc_merged_left)
                            proc_merged_left <= scan_work_left;
                        if (scan_work_right > proc_merged_right)
                            proc_merged_right <= scan_work_right;
                        if (scan_work_top < proc_merged_top)
                            proc_merged_top <= scan_work_top;
                        if (scan_work_bottom > proc_merged_bottom)
                            proc_merged_bottom <= scan_work_bottom;
                        proc_merged_area <= proc_merged_area + scan_work_area;
                    end

                    if (proc_scan_index == OIW'(MAX_OBJECTS - 1))
                        proc_state <= PROC_COMMIT;
                    else begin
                        proc_scan_index <= proc_scan_index + 1'b1;
                        proc_state <= PROC_LOAD;
                    end
                end
                default: proc_state <= PROC_IDLE;
            endcase

            if (stage1_fire && stage1_user) begin
                for (state_index = 0; state_index < MAX_OBJECTS; state_index++) begin
                    work_valid[state_index] <= 1'b0;
                    work_left[state_index] <= '0;
                    work_right[state_index] <= '0;
                    work_top[state_index] <= '0;
                    work_bottom[state_index] <= '0;
                    work_area[state_index] <= '0;
                end
            end else if (proc_state == PROC_COMMIT) begin
                for (state_index = 0; state_index < MAX_OBJECTS; state_index++) begin
                    if (proc_primary_valid) begin
                        if (proc_primary_index == OIW'(state_index)) begin
                            work_valid[state_index] <= 1'b1;
                            work_left[state_index] <= proc_merged_left;
                            work_right[state_index] <= proc_merged_right;
                            work_top[state_index] <= proc_merged_top;
                            work_bottom[state_index] <= proc_merged_bottom;
                            work_area[state_index] <= proc_merged_area;
                        end else if (proc_match_mask[state_index]) begin
                            work_valid[state_index] <= 1'b0;
                            work_area[state_index] <= '0;
                        end
                    end else if (proc_free_valid &&
                                 (proc_free_index == OIW'(state_index))) begin
                        work_valid[state_index] <= 1'b1;
                        work_left[state_index] <= proc_start;
                        work_right[state_index] <= proc_end;
                        work_top[state_index] <= proc_y;
                        work_bottom[state_index] <= proc_y;
                        work_area[state_index] <= AW'(proc_length);
                    end else if (!proc_free_valid &&
                                 (AW'(proc_length) > proc_smallest_area) &&
                                 (proc_smallest_index == OIW'(state_index))) begin
                        work_valid[state_index] <= 1'b1;
                        work_left[state_index] <= proc_start;
                        work_right[state_index] <= proc_end;
                        work_top[state_index] <= proc_y;
                        work_bottom[state_index] <= proc_y;
                        work_area[state_index] <= AW'(proc_length);
                    end
                end
            end

            if (stage1_fire && stage1_last &&
                (stage1_pixel_y == YW'(V_RES - 1)))
                frame_end_pending <= 1'b1;

            if (frame_end_pending && (proc_state == PROC_IDLE) &&
                (run_fifo_count == 0) && !run_enqueue) begin
                frame_end_pending <= 1'b0;
                snapshot_pending <= 1'b1;
            end

            if (snapshot_pending && !eval_active && !commit_active) begin
                snapshot_pending <= 1'b0;
                input_frame_hold <= 1'b0;
                eval_active <= 1'b1;
                eval_phase <= EVAL_LOAD;
                eval_index <= '0;
                pending_count <= '0;
                for (state_index = 0; state_index < MAX_OBJECTS; state_index++) begin
                    eval_valid[state_index] <= work_valid[state_index];
                    eval_left[state_index] <= work_left[state_index];
                    eval_right[state_index] <= work_right[state_index];
                    eval_top[state_index] <= work_top[state_index];
                    eval_bottom[state_index] <= work_bottom[state_index];
                    eval_area[state_index] <= work_area[state_index];
                    pending_valid[state_index] <= 1'b0;
                end
            end

            if (eval_active) begin
                case (eval_phase)
                    EVAL_LOAD: begin
                        candidate_source_valid <= eval_valid[eval_index];
                        candidate_left <= eval_left[eval_index];
                        candidate_right <= eval_right[eval_index];
                        candidate_top <= eval_top[eval_index];
                        candidate_bottom <= eval_bottom[eval_index];
                        candidate_width <= {1'b0, eval_right[eval_index]} -
                                           {1'b0, eval_left[eval_index]} + 1'b1;
                        candidate_height <= {1'b0, eval_bottom[eval_index]} -
                                            {1'b0, eval_top[eval_index]} + 1'b1;
                        candidate_center_x <= (eval_left[eval_index] >> 1) +
                                              (eval_right[eval_index] >> 1) +
                                              (eval_left[eval_index][0] & eval_right[eval_index][0]);
                        candidate_center_y <= (eval_top[eval_index] >> 1) +
                                              (eval_bottom[eval_index] >> 1) +
                                              (eval_top[eval_index][0] & eval_bottom[eval_index][0]);
                        candidate_area <= eval_area[eval_index];
                        eval_phase <= EVAL_MULTIPLY;
                    end
                    EVAL_MULTIPLY: begin
                        candidate_bbox_area <= AW'(candidate_width * candidate_height);
                        eval_phase <= EVAL_ANALYZE;
                    end
                    default: begin
                        if (candidate_shape_valid && (pending_count < MAX_OBJECTS)) begin
                            for (state_index = 0; state_index < MAX_OBJECTS; state_index++) begin
                                if (pending_count == OCW'(state_index)) begin
                                    pending_valid[state_index] <= 1'b1;
                                    pending_left[state_index] <= candidate_left;
                                    pending_right[state_index] <= candidate_right;
                                    pending_top[state_index] <= candidate_top;
                                    pending_bottom[state_index] <= candidate_bottom;
                                    pending_center_x[state_index] <= candidate_center_x;
                                    pending_center_y[state_index] <= candidate_center_y;
                                end
                            end
                            pending_count <= pending_count + 1'b1;
                        end

                        if (eval_index == OIW'(MAX_OBJECTS - 1)) begin
                            eval_active <= 1'b0;
                            commit_active <= 1'b1;
                            commit_phase <= COMMIT_SELECT;
                            commit_update_enabled <= !pick_lock;
                            commit_index <= '0;
                            commit_count <= '0;
                        end else begin
                            eval_index <= eval_index + 1'b1;
                            eval_phase <= EVAL_LOAD;
                        end
                    end
                endcase
            end

            if (commit_active) begin
                case (commit_phase)
                    COMMIT_SELECT: begin
                        commit_pending_valid_reg <= pending_valid[commit_index];
                        commit_track_valid_reg <= track_valid[commit_index];
                        commit_pending_x_reg <= pending_center_x[commit_index];
                        commit_track_x_reg <= track_center_x[commit_index];
                        commit_pending_y_reg <= pending_center_y[commit_index];
                        commit_track_y_reg <= track_center_y[commit_index];
                        commit_track_stable_reg <= track_stable[commit_index];
                        commit_phase <= COMMIT_DISTANCE;
                    end
                    COMMIT_DISTANCE: begin
                        commit_x_near_reg <=
                            (((commit_pending_x_reg >= commit_track_x_reg) ?
                              (commit_pending_x_reg - commit_track_x_reg) :
                              (commit_track_x_reg - commit_pending_x_reg)) <= XW'(STABLE_TOL));
                        commit_y_near_reg <=
                            (((commit_pending_y_reg >= commit_track_y_reg) ?
                              (commit_pending_y_reg - commit_track_y_reg) :
                              (commit_track_y_reg - commit_pending_y_reg)) <= YW'(STABLE_TOL));
                        commit_phase <= COMMIT_CALC;
                    end
                    COMMIT_CALC: begin
                        commit_calc_stable <= commit_slot_stable;
                        commit_calc_confirmed <= commit_slot_confirmed;
                        commit_phase <= COMMIT_STORE;
                    end
                    default: begin
                        if (commit_update_enabled) begin
                            if (commit_calc_confirmed)
                                commit_count <= commit_count + 1'b1;

                            // Constant destinations avoid a wide dynamic-write mux.
                            for (state_index = 0; state_index < MAX_OBJECTS; state_index++) begin
                                if (commit_index == OIW'(state_index)) begin
                                    track_valid[state_index] <= pending_valid[state_index];
                                    track_center_x[state_index] <= pending_center_x[state_index];
                                    track_center_y[state_index] <= pending_center_y[state_index];
                                    track_stable[state_index] <= commit_calc_stable;
                                    box_valid[state_index] <= commit_calc_confirmed;
                                    box_left[state_index] <= pending_left[state_index];
                                    box_right[state_index] <= pending_right[state_index];
                                    box_top[state_index] <= pending_top[state_index];
                                    box_bottom[state_index] <= pending_bottom[state_index];
                                    box_center_x[state_index] <= pending_center_x[state_index];
                                    box_center_y[state_index] <= pending_center_y[state_index];
                                    box_stable[state_index] <= commit_calc_stable;
                                end
                            end
                        end

                        if (commit_index == OIW'(MAX_OBJECTS - 1)) begin
                            commit_active <= 1'b0;
                            frame_done <= 1'b1;
                            if (commit_update_enabled)
                                object_count <= commit_count + commit_calc_confirmed;
                        end else begin
                            commit_index <= commit_index + 1'b1;
                            commit_phase <= COMMIT_SELECT;
                        end
                    end
                endcase
            end

            if (!bcd_active && bcd_sample_valid &&
                ((bcd_sample_x != bcd_last_x) || (bcd_sample_y != bcd_last_y) ||
                 (bcd_sample_select != bcd_last_select))) begin
                bcd_active <= 1'b1;
                bcd_count <= '0;
                bcd_x_bin <= bcd_sample_x;
                bcd_y_bin <= bcd_sample_y;
                bcd_select_bin <= bcd_sample_select;
                bcd_x_work <= '0;
                bcd_y_work <= '0;
            end else if (bcd_active) begin
                bcd_next_x = bcd_shift(bcd_x_work, bcd_x_bin[XW - 1 - bcd_count]);
                bcd_next_y = bcd_shift(bcd_y_work, bcd_y_bin[YW - 1 - bcd_count]);
                bcd_x_work <= bcd_next_x;
                bcd_y_work <= bcd_next_y;
                if (bcd_count == BCW'(BW - 1)) begin
                    bcd_active <= 1'b0;
                    bcd_last_x <= bcd_x_bin;
                    bcd_last_y <= bcd_y_bin;
                    bcd_last_select <= bcd_select_bin;
                    coord_x_d3 <= bcd_next_x[15:12];
                    coord_x_d2 <= bcd_next_x[11:8];
                    coord_x_d1 <= bcd_next_x[7:4];
                    coord_x_d0 <= bcd_next_x[3:0];
                    coord_y_d3 <= bcd_next_y[15:12];
                    coord_y_d2 <= bcd_next_y[11:8];
                    coord_y_d1 <= bcd_next_y[7:4];
                    coord_y_d0 <= bcd_next_y[3:0];
                end else begin
                    bcd_count <= bcd_count + 1'b1;
                end
            end
        end
    end
endmodule
