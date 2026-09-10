// ============================================================================
// video_axis_packer — 将 video_display 的像素流实时打包为 128-bit AXI-Stream
//
// 数据流:
//   video_display  →  video_axis_packer  →  CDC async_fifo  →  DDR3
//   (16-bit RGB565)    (128-bit AXI-S)       (跨时钟域)
//
// 打包规则:
//   - 每像素 16-bit = RGB565 (直接透传, 不转换)
//   - 每 8 个像素打包为 1 拍 128-bit
//   - pixel_0（先到）放在 [15:0]，pixel_7（后到）放在 [127:112]
//   - Host 端按顺序读: byte0-1=pixel_0, byte2-3=pixel_1, ...
//
// 帧同步:
//   - 检测 video_vs 上升沿 → 重置内部计数器（新帧开始）
//   - 每帧固定 115,200 拍 (1280×720/8)
//   - 最后一拍 (beat_cnt == 115,199) 带 tlast=1
//
// 反压处理:
//   - downstream tready=0 时，停在 pixel_cnt==7，等 tready 恢复
//   - pixel_cnt!=7 时正常累积，不受反压影响（最多等 7 个像素）
//
// 时钟域: pixel_clk (75 MHz)
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module video_axis_packer (
    input  wire         clk,            // pixel_clk, 75MHz
    input  wire         rst_n,          // 低有效复位

    // === 视频输入 (从 video_driver) ===
    input  wire         video_de,       // 像素有效指示
    input  wire         video_vs,       // 场同步 (上升沿=帧开始)
    input  wire [15:0]  pixel_data,     // RGB565 像素数据

    // === AXI-Stream 输出 (128-bit) ===
    output reg  [127:0] m_axis_tdata,   // 打包后的 4 像素数据
    output reg  [15:0]  m_axis_tkeep,   // 字节有效标志 (固定 16'hFFFF)
    output reg          m_axis_tvalid,  // 输出有效
    input  wire         m_axis_tready,  // 下游准备好
    output reg          m_axis_tlast    // 帧尾标记
);

    // ========================================================================
    // 参数
    // ========================================================================
    // 1280 × 720 分辨率，每拍 8 像素 (16-bit RGB565)
    localparam PIXELS_PER_BEAT = 8;
    localparam BEATS_PER_FRAME = (1280 * 720) / PIXELS_PER_BEAT;  // 115,200
    localparam BEAT_CNT_WIDTH  = $clog2(BEATS_PER_FRAME);         // 18 bits

    // ========================================================================
    // 内部信号
    // ========================================================================
    reg [2:0]              pixel_cnt;       // 当前拍累积了几个像素 (0~7)
    reg [BEAT_CNT_WIDTH-1:0] beat_cnt;      // 当前帧已发出的拍数
    reg                    video_vs_d1;     // vsync 打一拍，用于边沿检测
    reg [111:0]            pix_accum;       // 前 7 个像素的缓冲 (7×16bit)
    reg                    frame_drop;      // ★ 反压丢帧标志 (摄像头不能 stall)

    // 边沿检测：vsync 上升沿 = 新帧开始
    wire vsync_rising = video_vs && !video_vs_d1;

    // 下游能否接收新数据：当前拍已被取走 或 还没有待发送的数据
    wire can_output = !m_axis_tvalid || m_axis_tready;

    // ★ v1.1 反压: 已攒满8像素但下游不收 → 丢当前帧 (不暂停视频源)
    //   摄像头不能 stall, 反压只能丢整帧 (vsync 重新对齐)
    wire backpressure = (pixel_cnt == 3'd7) && !can_output && video_de;

    // ========================================================================
    // 主逻辑
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_cnt     <= 3'd0;
            beat_cnt      <= {BEAT_CNT_WIDTH{1'b0}};
            video_vs_d1   <= 1'b0;
            pix_accum     <= 112'd0;
            m_axis_tdata  <= 128'd0;
            m_axis_tkeep  <= 16'd0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            frame_drop    <= 1'b0;
        end else begin
            // ----- 延迟一拍 vsync，用于边沿检测 -----
            video_vs_d1 <= video_vs;

            // ----- 新帧开始：复位计数器 + 清除丢帧 -----
            // vsync 上升沿表示 VSYNC 结束，新一帧有效像素即将到来
            if (vsync_rising) begin
                pixel_cnt  <= 3'd0;
                beat_cnt   <= {BEAT_CNT_WIDTH{1'b0}};
                pix_accum  <= 112'd0;
                frame_drop <= 1'b0;   // 新帧重新对齐
            end

            // ----- 反压检测: 满4像素发不出 → 丢当前帧 -----
            if (backpressure)
                frame_drop <= 1'b1;

            // ----- 清除已成拍的 valid（下游已取走） -----
            if (m_axis_tvalid && m_axis_tready) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end

            // ----- 逐个像素打包 (frame_drop=1 时丢弃整帧) -----
            if (video_de && !frame_drop) begin

                if (pixel_cnt == 3'd7) begin
                    // ======================================================
                    // 第 8 个像素到达: 当前拍凑齐，准备输出
                    // 格式: {pixel_7(新), pixel_6, ..., pixel_0(旧)}
                    // ======================================================
                    if (can_output) begin
                        // 下游有空 → 发出
                        m_axis_tdata  <= {pixel_data, pix_accum};
                        m_axis_tkeep  <= 16'hFFFF;                  // 全场满
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (beat_cnt == (BEATS_PER_FRAME - 1));  // 最后一拍
                        pixel_cnt     <= 3'd0;
                        beat_cnt      <= beat_cnt + 1'b1;
                        // pix_accum 内容已输出，下个像素从头累积
                        // （无需清零 pix_accum —— 下次会覆盖写入）
                    end
                    // else: 下游反压 → 停顿，保持 pixel_cnt==7 不更新
                    //       反压消除后下一拍 pixel_data 仍是同一组像素。

                end else begin
                    // ======================================================
                    // pixel_cnt = 0~6: 累积像素到对应位置
                    // pixel_n → accum[n*16 +: 16] (16-bit RGB565)
                    // ======================================================
                    pix_accum[pixel_cnt * 16 +: 16] <= pixel_data;
                    pixel_cnt <= pixel_cnt + 1'b1;
                end

            end  // if (video_de)
        end  // else (not reset)
    end  // always

endmodule

`default_nettype wire
