// ============================================================================
// video_frame_writer — 视频帧缓冲写入管理（10 帧环形缓冲）
//
// 职责:
//   1. 监听 CDC FIFO 输出的 AXI-Stream，检测帧边界
//   2. 管理 10 个帧槽位的基地址，循环写入
//   3. 每帧开始前向 axi_dma 发出写描述符（地址 + 长度）
//   4. 提供帧计数 / 当前写槽位 / 最新完整帧地址等状态
//
// 数据流:
//   CDC FIFO (ui_clk) ─→ axi_dma 的 s_axis_write_data_* 端口（直连，不经过本模块）
//   本模块 ─→ axi_dma 的 s_axis_write_desc_* 端口（描述符控制）
//
// 设计要点:
//   - 本模块不碰像素数据，只生成描述符
//   - 描述符在每帧第一拍数据到来时发出（frame_base + idx * stride）
//   - 帧尾 tlast=1 时 → 切换到下一个槽位
//   - 10 帧环形：idx = 0→1→2→...→9→0→1...
//   - 提供 latest_addr = 最新已完整缓存的帧的 DDR 起始地址
//   - Host 读取逻辑：永远读取 (write_idx + 5) % 10 槽位（滞后 5 帧，避开正在写的帧和 posted write）
//
// CDC:
//   - cfg_* 输入来自 tlp_clk 域 (logic_dma BAR2)，内部 2-FF 同步到 ui_clk
//   - status_* 输出到 tlp_clk 域，在输出端口直接给出（调用方负责同步）
//
// 时钟域: ui_clk (DDR3 AXI 互联时钟)
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module video_frame_writer #(
    // ========================================================================
    // 参数
    // ========================================================================
    parameter NUM_FRAMES      = 10,             // 缓冲帧数 (10 槽环形)
    parameter FRAME_IDX_WIDTH = $clog2(NUM_FRAMES),  // 4 bits for 10 frames
    parameter AXI_ADDR_WIDTH  = 28,            // AXI 地址位宽
    parameter AXI_LEN_WIDTH   = 29,            // 描述符长度位宽 (=ADDR_WIDTH+1)
    parameter FRAME_CNT_WIDTH = 16             // 帧计数器位宽
) (
    // ========================================================================
    // 时钟 & 复位
    // ========================================================================
    input  wire         clk,                    // ui_clk (DDR3 域)
    input  wire         rst_n,                  // 低有效复位

    // ========================================================================
    // 配置输入（来自 tlp_clk 域的 logic_dma，需同步）
    //   这些信号在 Host 初始化时写入一次，之后基本不变
    //   本模块内部做 2-FF 同步
    // ========================================================================
    input  wire                        cfg_enable,       // 使能 (1=开始写帧)
    input  wire [AXI_ADDR_WIDTH-1:0]   cfg_frame_base,   // 帧缓冲区 DDR 基地址
    input  wire [AXI_LEN_WIDTH-1:0]    cfg_frame_stride,  // 单帧字节跨度

    // ========================================================================
    // AXI-Stream 监控（来自 CDC FIFO 输出）
    //   只收 tvalid + tlast，不收 tdata（数据直连 axi_dma）
    // ========================================================================
    input  wire         s_axis_tvalid,     // 输入数据有效
    input  wire         s_axis_tlast,      // 输入帧尾标记

    // ========================================================================
    // AXI 写描述符输出（→ axi_dma 的描述符输入端口）
    // ========================================================================
    output reg  [AXI_ADDR_WIDTH-1:0] m_axis_write_desc_addr,
    output reg  [AXI_LEN_WIDTH-1:0]  m_axis_write_desc_len,
    output reg                       m_axis_write_desc_valid,
    input  wire                      m_axis_write_desc_ready,

    // ========================================================================
    // 状态输出（到 tlp_clk 域 logic_dma，调用方负责同步）
    // ========================================================================
    output wire [FRAME_IDX_WIDTH-1:0] status_write_idx,     // 当前写入槽位
    output wire [FRAME_CNT_WIDTH-1:0] status_frame_count,   // 已完成帧总数
    output wire [AXI_ADDR_WIDTH-1:0]  status_latest_addr    // 最新完整帧地址
);

    // ========================================================================
    // CDC：2-FF 同步器 — 将 tlp_clk 域的配置信号同步到 ui_clk
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg cfg_enable_s1;
    (* ASYNC_REG = "TRUE" *) reg cfg_enable_s2;
    (* ASYNC_REG = "TRUE" *) reg [AXI_ADDR_WIDTH-1:0] cfg_frame_base_s1;
    (* ASYNC_REG = "TRUE" *) reg [AXI_ADDR_WIDTH-1:0] cfg_frame_base_s2;
    (* ASYNC_REG = "TRUE" *) reg [AXI_LEN_WIDTH-1:0]  cfg_frame_stride_s1;
    (* ASYNC_REG = "TRUE" *) reg [AXI_LEN_WIDTH-1:0]  cfg_frame_stride_s2;

    wire enable       = cfg_enable_s2;        // 同步后的使能
    wire [AXI_ADDR_WIDTH-1:0] frame_base  = cfg_frame_base_s2;
    wire [AXI_LEN_WIDTH-1:0]  frame_stride = cfg_frame_stride_s2;

    always @(posedge clk) begin
        // 2 级同步寄存器链
        {cfg_enable_s2,       cfg_enable_s1}       <= {cfg_enable_s1,       cfg_enable};
        {cfg_frame_base_s2,   cfg_frame_base_s1}   <= {cfg_frame_base_s1,   cfg_frame_base};
        {cfg_frame_stride_s2, cfg_frame_stride_s1} <= {cfg_frame_stride_s1, cfg_frame_stride};
    end

    // ========================================================================
    // 内部寄存器
    // ========================================================================
    reg [FRAME_IDX_WIDTH-1:0] write_idx;      // 当前写入槽位 (0-9)
    reg [FRAME_CNT_WIDTH-1:0] frame_count;     // 已完成帧总数（不包含正在写的）
    reg [AXI_ADDR_WIDTH-1:0]  latest_addr;     // 最新完整帧的地址
    reg                       desc_sent;       // 当前帧的描述符已发出

    // ========================================================================
    // 状态机
    // ========================================================================
    localparam IDLE       = 2'd0;   // 等待新帧
    localparam ISSUE_DESC = 2'd1;   // 发出写描述符
    localparam TRACKING   = 2'd2;   // 帧正在写入中，等 tlast

    reg [1:0] state;
    reg [1:0] state_next;

    // ---- 帧开始检测 ----
    // 条件: 使能 + 有数据到来 + 当前不在帧中 (desc_sent=0)
    wire frame_start = enable && s_axis_tvalid && !desc_sent && !s_axis_tlast;

    // ---- 描述符握手 ----
    wire desc_accepted = m_axis_write_desc_valid && m_axis_write_desc_ready;

    // ---- 帧尾检测 ----
    wire frame_end = s_axis_tvalid && s_axis_tlast && desc_sent;

    // ========================================================================
    // 状态转移（组合逻辑）
    // ========================================================================
    always @(*) begin
        state_next = state;

        case (state)
            IDLE: begin
                if (frame_start)
                    state_next = ISSUE_DESC;
            end

            ISSUE_DESC: begin
                if (desc_accepted)
                    state_next = TRACKING;
            end

            TRACKING: begin
                if (frame_end)
                    state_next = IDLE;
            end

            default: state_next = IDLE;
        endcase
    end

    // ========================================================================
    // 描述符计算（组合逻辑）
    // ========================================================================
    // 当前帧的 DDR 地址 = 基地址 + 当前槽位 × 跨度
    wire [AXI_ADDR_WIDTH-1:0] frame_addr;
    assign frame_addr = frame_base + (write_idx * frame_stride[AXI_ADDR_WIDTH-1:0]);

    // ========================================================================
    // 时序逻辑
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                    <= IDLE;
            write_idx                <= {FRAME_IDX_WIDTH{1'b0}};
            frame_count              <= {FRAME_CNT_WIDTH{1'b0}};
            latest_addr              <= {AXI_ADDR_WIDTH{1'b0}};
            desc_sent                <= 1'b0;
            m_axis_write_desc_valid  <= 1'b0;
            m_axis_write_desc_addr   <= {AXI_ADDR_WIDTH{1'b0}};
            m_axis_write_desc_len    <= {AXI_LEN_WIDTH{1'b0}};
        end else begin
            state <= state_next;

            // ----- 描述符握手完成 → 清除 valid -----
            if (desc_accepted) begin
                m_axis_write_desc_valid <= 1'b0;
            end

            // ----- 状态转移动作 -----
            case (state)
                IDLE: begin
                    // 不使能时，重置内部状态（让 Host 可以重新配置）
                    if (!enable) begin
                        write_idx   <= {FRAME_IDX_WIDTH{1'b0}};
                        frame_count <= {FRAME_CNT_WIDTH{1'b0}};
                        latest_addr <= {AXI_ADDR_WIDTH{1'b0}};
                        desc_sent   <= 1'b0;
                    end

                    // 检测到帧开始 → 发描述符
                    if (frame_start) begin
                        m_axis_write_desc_addr  <= frame_addr;
                        m_axis_write_desc_len   <= frame_stride;
                        m_axis_write_desc_valid <= 1'b1;
                        desc_sent               <= 1'b1;
                    end
                end

                ISSUE_DESC: begin
                    // 等待 axi_dma 接收描述符（valid 在上面的 desc_accepted 中清除）
                end

                TRACKING: begin
                    // 帧尾到达 → 切换到下一个槽位
                    if (frame_end) begin
                        // 更新状态：最新完整帧 = 当前帧的地址
                        latest_addr <= frame_addr;
                        // 槽位轮转: 0→1→2→0
                        if (write_idx == (NUM_FRAMES - 1))
                            write_idx <= {FRAME_IDX_WIDTH{1'b0}};
                        else
                            write_idx <= write_idx + 1'b1;
                        // 帧计数 +1
                        frame_count <= frame_count + 1'b1;
                        // 准备接收下一帧
                        desc_sent <= 1'b0;
                    end
                end

                default: ;
            endcase
        end
    end

    // ========================================================================
    // 状态输出
    // ========================================================================
    assign status_write_idx   = write_idx;
    assign status_frame_count = frame_count;
    assign status_latest_addr = latest_addr;

endmodule

`default_nettype wire
