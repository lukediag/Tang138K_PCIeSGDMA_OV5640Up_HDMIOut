// ============================================================================
// axis_burst_gate — CDC FIFO 水位门控 AXI-Stream
//
// 用途:
//   在 Video CDC FIFO 和 axi_dma_video 之间加一级门控，
//   让 Video 写 DDR3 从"连续小流量"变成"间歇大 burst"，
//   从而给 PCIe C2H 读出仲裁窗口。
//
// 工作原理:
//   - 监控 CDC FIFO 输出端水位 (m_status_depth)
//   - 水位 ≥ HIGH_WATER → gate_open=1, 数据流向下游 (axi_dma_video)
//   - 水位 ≤ LOW_WATER  → gate_open=0, 暂停输出, FIFO 重新积累
//   - tlast 帧尾: 如果在 gate_open 期间到达, 保持 gate_open
//     直到该帧数据全部排出 (防止帧尾残留在 FIFO 中)
//
// 水位参数 (针对 DEPTH=16384):
//   HIGH_WATER = 8192  (50%)  — 攒够半满才写
//   LOW_WATER  = 1024  (6.25%) — 排空大部分后关门
//
// 时钟域: 与 CDC FIFO 输出端相同 (ui_clk)
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module axis_burst_gate #(
    parameter FIFO_DEPTH  = 16384,               // CDC FIFO 深度
    parameter HIGH_WATER  = FIFO_DEPTH / 2,      // 8192 (50%)
    parameter LOW_WATER   = FIFO_DEPTH / 16,     // 1024 (6.25%)
    parameter DATA_WIDTH  = 128,                 // AXI-Stream 数据宽度
    parameter KEEP_WIDTH  = 16,                  // tkeep 宽度
    parameter DEPTH_WIDTH = $clog2(FIFO_DEPTH+1) // 水位信号位宽
) (
    input  wire                     clk,
    input  wire                     rst_n,

    // ---- 水位输入 (来自 axis_async_fifo.m_status_depth) ----
    input  wire [DEPTH_WIDTH-1:0]   fifo_depth,

    // ---- AXI-Stream 输入 (来自 CDC FIFO 输出) ----
    input  wire [DATA_WIDTH-1:0]    s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]    s_axis_tkeep,
    input  wire                     s_axis_tvalid,
    output wire                     s_axis_tready,
    input  wire                     s_axis_tlast,

    // ---- AXI-Stream 输出 (→ axi_dma_video + video_frame_writer) ----
    output wire [DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [KEEP_WIDTH-1:0]    m_axis_tkeep,
    output wire                     m_axis_tvalid,
    input  wire                     m_axis_tready,
    output wire                     m_axis_tlast
);

    // ========================================================================
    // 门控状态机
    // ========================================================================
    reg gate_open;
    reg drain_mode;   // 强制排空: 看到 tlast 后, 保持 OPEN 直到帧尾通过

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_open  <= 1'b0;
            drain_mode <= 1'b0;
        end else begin
            // ---- drain_mode: gate_open 期间收到 tlast, 标记本帧需排空 ----
            if (gate_open && s_axis_tvalid && s_axis_tlast && m_axis_tready) begin
                // tlast 本周期会通过 gate → 帧尾已排出
                // 但同一帧可能还有后续数据在 FIFO 中? 不会,
                // 因为 tlast 是帧的最后一个 beat, 之后就是下一帧的数据
                drain_mode <= 1'b0;
            end else if (gate_open && s_axis_tvalid && s_axis_tlast && !m_axis_tready) begin
                // tlast 被下游反压, 还没通过 → 保持 drain 直到它通过
                drain_mode <= 1'b1;
            end

            // ---- gate_open: 水位控制 + drain_mode 覆盖 ----
            if (drain_mode) begin
                // 排空中: 保持 gate_open, 直到 FIFO 空
                if (fifo_depth <= 1)
                    drain_mode <= 1'b0;
                // gate_open stays 1
            end else if (gate_open && fifo_depth <= LOW_WATER) begin
                // 水位降到低位 → 关门, 让 FIFO 重新积累
                gate_open <= 1'b0;
            end else if (!gate_open && fifo_depth >= HIGH_WATER) begin
                // 水位到达高位 → 开门, 开始 burst
                gate_open <= 1'b1;
            end
        end
    end

    // ========================================================================
    // 数据路径: 直通 + 门控
    // ========================================================================
    // 数据/tkeep/tlast 始终直通, 因为 valid=0 时下游不会采样
    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tkeep  = s_axis_tkeep;
    assign m_axis_tlast  = s_axis_tlast;

    // 门控 valid: 只在 gate_open 时向下游暴露数据
    assign m_axis_tvalid = s_axis_tvalid && gate_open;

    // 门控 ready: 只在 gate_open 时向上游传递 backpressure
    //   关门时 ready=0 → CDC FIFO 停止输出 → 数据在 FIFO 中积累
    assign s_axis_tready = gate_open && m_axis_tready;

endmodule

`default_nettype wire
