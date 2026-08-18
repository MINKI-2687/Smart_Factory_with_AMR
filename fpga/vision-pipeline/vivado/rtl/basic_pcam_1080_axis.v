`timescale 1ns / 1ps

// Vivado 2020.2 module-reference wrapper for the Zybo Z7 Pcam 5C 1080p pipeline.
// coord_word bit map: [10:0] X, [21:11] Y, [22] valid,
// [25:23] object_count, [28:26] selected index, [31:29] stable count.
module basic_pcam_1080_axis (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
    input wire aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input wire aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input wire [23:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input wire s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input wire s_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input wire s_axis_tlast,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    output wire [23:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output wire m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input wire m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TUSER" *)
    output wire m_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output wire m_axis_tlast,

    input wire [2:0] coord_select,
    output wire [31:0] coord_word
);
    wire coord_valid;
    wire [10:0] coord_x;
    wire [10:0] coord_y;
    wire [2:0] object_count;
    wire frame_done;

    basic_axis_blue_tracker_1080 #(
        .H_RES(1920),
        .V_RES(1080),
        // Broad temporary work area. Calibrate these values after mounting the camera.
        .PICK_X0(40),
        .PICK_Y0(40),
        .PICK_X1(1879),
        .PICK_Y1(1039),
        .MAX_OBJECTS(6),
        .MIN_FILL_SHIFT(3),
        .STABLE_FRAMES(3),
        .STABLE_TOL(24),
        // Initial wide blue range. Tune here after testing the mounted camera.
        .HSV_HUE_MIN(135),
        .HSV_HUE_MAX(200),
        .HSV_SAT_MIN(70),
        .HSV_VALUE_MIN(40),
        .PICK_BORDER_ENABLE(1'b0),
        .OSD_ENABLE(1'b1)
    ) basic_axis_blue_tracker_1080_i (
        .aclk(aclk),
        .aresetn(aresetn),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),
        .s_axis_tuser(s_axis_tuser),
        .s_axis_tlast(s_axis_tlast),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),
        .m_axis_tuser(m_axis_tuser),
        .m_axis_tlast(m_axis_tlast),
        .pick_lock(1'b0),
        .coord_select(coord_select),
        .coord_valid(coord_valid),
        .coord_x(coord_x),
        .coord_y(coord_y),
        .object_count(object_count),
        .frame_done(frame_done),
        .coord_word(coord_word)
    );
endmodule
