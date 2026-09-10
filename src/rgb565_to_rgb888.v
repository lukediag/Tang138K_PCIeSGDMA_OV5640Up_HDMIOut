// ============================================================================
// rgb565_to_rgb888 — OV5640 RGB565 → RGB888 位扩展转换 (纯组合逻辑)
//
// 转换规则 (与 36_ov5640_hdmi 例程 hdmi_top.v 同款低位补 0 插值):
//   R = RGB565[15:11] << 3   (5 bit → 8 bit)
//   G = RGB565[10:5]  << 2   (6 bit → 8 bit)
//   B = RGB565[4:0]   << 3   (5 bit → 8 bit)
//
// 时钟域: 跟随上游 cmos_capture_data (cam_pclk 域), 无寄存器
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module rgb565_to_rgb888 (
    input  wire [15:0] s_data,     // RGB565 输入 (cmos_frame_data)
    input  wire        s_valid,    // 数据有效 (cmos_frame_valid)
    input  wire        s_vsync,    // 场同步 (cmos_frame_vsync)

    output wire [23:0] m_data,     // RGB888 输出 (低 24 bit 有效)
    output wire        m_valid,    // 数据有效 (透传)
    output wire        m_vsync     // 场同步 (透传)
);

    // RGB565 → RGB888 位扩展 (低位补 0)
    assign m_data  = {s_data[15:11], 3'd0,   // R: 5bit → 8bit
                      s_data[10: 5], 2'd0,   // G: 6bit → 8bit
                      s_data[ 4: 0], 3'd0};  // B: 5bit → 8bit

    assign m_valid = s_valid;
    assign m_vsync = s_vsync;

endmodule

`default_nettype wire
