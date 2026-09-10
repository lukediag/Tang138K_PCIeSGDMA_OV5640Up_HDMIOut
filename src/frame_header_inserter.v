// ============================================================================
// frame_header_inserter v2.0 — 帧头 + 帧尾插入 + 溢出全F填充 (DDR3 bypass)
//
// ★ v2.0 改动 (vs v1.x 仅帧头):
//   - 帧头新格式: AA AB AC AD + 预计行数 + 预计列数 + 帧ID
//   - 帧尾新格式: FA FB FC FD + xx(帧状态) + 行数状态 + 列数状态 + 帧ID
//   - 溢出处理: 溢出后输出全F + 错误帧尾(E1) → SGDMA 按 desc.length 收满不挂起
//
// 帧格式 (总 1,843,232 B = 帧头16 + 像素1,843,200 + 帧尾16):
//   ┌──────────┬──────────────────────┬──────────┐
//   │ 帧头 16B │ 像素 1,843,200 B     │ 帧尾 16B │
//   └──────────┴──────────────────────┴──────────┘
//
// 帧头 128-bit (Verilog 拼接, SGDMA 按小端写内存):
//   [127:96]=magic(拼接值 0xADACABAA) [95:80]=预计行数(720) [79:64]=预计列数(1280)
//   [63:48]=frame_id [47:0]=reserved
//   → 内存字节 12..15=AA AB AC AD, 10..11=行数, 8..9=列数, 6..7=帧ID
// 帧尾 128-bit (Verilog 拼接, SGDMA 按小端写内存):
//   [127:96]=magic(拼接值 0xFDFCFBFA) [95:88]=xx(01正常/E1溢出) [87:80]=行数状态
//   [79:72]=列数状态 [71:56]=frame_id [55:0]=reserved
//   → 内存字节 12..15=FA FB FC FD, 11=xx, 10=行状态, 9=列状态, 7..8=帧ID
//
// 溢出机制:
//   - overflow_in = CDC FIFO 写侧 full 的 CDC 到读侧电平标志
//   - frame_overflowed: 帧内溢出锁存 (SOF 清零, overflow_in 置位, 帧尾读取)
//   - 溢出后输出全F + E1 帧尾 → SGDMA 正常完成 desc 不挂起
//   - 溢出时 FIFO 不清 (由 cam_vs 每帧清), 只标记错误帧尾
//
// 时钟域: tlp_clk (100MHz, 与 SGDMA C2H 同域)
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module frame_header_inserter #(
    parameter DATA_WIDTH   = 128,               // AXI-Stream 数据位宽
    parameter KEEP_WIDTH   = DATA_WIDTH / 8,    // tkeep 位宽 = 16
    parameter FRAME_BYTES  = 3686400,           // 1280×720×4 像素字节
    parameter HEADER_BYTES = 16,                // 帧头字节
    parameter TAIL_BYTES   = 16,                // 帧尾字节
    parameter FRAME_ID_W   = 16,                // 帧 ID 位宽
    parameter EXPECTED_ROWS = 720,              // ★ v2.0: 预计行数 (写入帧头)
    parameter EXPECTED_COLS = 1280              // ★ v2.0: 预计列数 (写入帧头)
) (
    input  wire                     clk,
    input  wire                     rst,          // 高有效复位

    // ---- AXI-Stream 输入 (来自 CDC FIFO 输出端) ----
    input  wire [DATA_WIDTH-1:0]    s_axis_tdata,
    input  wire [KEEP_WIDTH-1:0]    s_axis_tkeep,
    input  wire                     s_axis_tvalid,
    output wire                     s_axis_tready,
    input  wire                     s_axis_tlast,

    // ---- AXI-Stream 输出 (→ axis_width_up → SGDMA C2H) ----
    output wire [DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [KEEP_WIDTH-1:0]    m_axis_tkeep,
    output wire                     m_axis_tvalid,
    input  wire                     m_axis_tready,
    output wire                     m_axis_tlast,

    // ---- ★ v2.0: 溢出标志 (读侧, 来自 CDC FIFO 写侧 full) ----
    input  wire                     overflow_in,   // 1=本帧发生溢出

    // ---- ★ v2.0: 行数/列数状态 (来自 cam_pclk 域检测, 已 CDC) ----
    input  wire                     row_status,    // 1=有效行数≠预计行数
    input  wire                     col_status,    // 1=某行列数≠预计列数

    // ---- ★ v2.0: 本帧进行中 (供写侧 gate FSM 判断 SGDMA 是否取完) ----
    input  wire [FRAME_ID_W-1:0]    frame_id,       // 递增帧号 (供帧头/帧尾写入)
    output reg                      frame_active    // SOF→tlast 之间为 1
);

    // ========================================================================
    // 参数派生
    // ========================================================================
    localparam integer FRAME_TOTAL_BYTES = FRAME_BYTES + HEADER_BYTES + TAIL_BYTES; // 1,843,232
    localparam integer BYTES_PER_BEAT    = DATA_WIDTH / 8;                          // 16
    localparam integer TAIL_POS          = FRAME_TOTAL_BYTES - TAIL_BYTES;          // 1,843,216 帧尾前位置
    localparam integer CNT_WIDTH         = 23;                                      // 容纳 1,843,232

    // ========================================================================
    // 状态机
    // ========================================================================
    localparam S_IDLE          = 3'd0;   // 等待新帧
    localparam S_INSERT_HEADER = 3'd1;   // 发送帧头 beat
    localparam S_FORWARD       = 3'd2;   // 转发像素 (含缓冲首拍)
    localparam S_FILL_DUMMY    = 3'd3;   // ★ 溢出后输出全F填充
    localparam S_INSERT_TAIL   = 3'd4;   // ★ 发送帧尾 beat (tlast)

    reg [2:0] state, state_next;

    // ========================================================================
    // 内部寄存器
    // ========================================================================
    reg [DATA_WIDTH-1:0]    buf_tdata;      // 缓冲的首个像素 beat
    reg [KEEP_WIDTH-1:0]    buf_tkeep;
    reg                     buf_tlast;      // 缓冲 beat 的 tlast (正常=0)
    reg                     buf_valid;      // buf 中有待发送数据
    // reg [FRAME_ID_W-1:0]    frame_id;       // 递增帧号
    reg                     frame_overflowed; // ★ 本帧溢出锁存
    reg [CNT_WIDTH-1:0]     byte_cnt;       // ★ 本帧已输出字节数

    // ========================================================================
    // 帧头 / 帧尾拼接
    // ========================================================================
    wire [DATA_WIDTH-1:0] header;
    assign header = {
        32'hADACABAA,                        // [127:96] Magic → 内存字节 12..15 = AA AB AC AD
        EXPECTED_ROWS[15:0],                 // [95:80]  预计行数 (内存字节 10..11, 小端)
        EXPECTED_COLS[15:0],                 // [79:64]  预计列数 (内存字节 8..9, 小端)
        frame_id,                            // [63:48]  帧ID (内存字节 6..7, 小端)
        48'd0                                // [47:0]   reserved
    };

    wire [DATA_WIDTH-1:0] tail;
    assign tail = {
        32'hFDFCFBFA,                                        // [127:96] 尾 magic → 内存字节 12..15 = FA FB FC FD
        frame_overflowed ? 8'hE1 : 8'h01,                    // [95:88]  xx: 帧状态 (E1=溢出, 01=正常) → 字节 11
        row_status       ? 8'hE1 : 8'h01,                    // [87:80]  行数状态 (E1=行数≠预计) → 字节 10
        col_status       ? 8'hE1 : 8'h01,                    // [79:72]  列数状态 (E1=列数≠预计) → 字节 9
        frame_id,                                            // [71:56]  帧ID (字节 7..8, 小端)
        56'd0                                                // [55:0]   reserved
    };

    // ========================================================================
    // SOF 检测: IDLE 状态下 tvalid 上升 → 新帧开始
    // ========================================================================
    wire sof = (state == S_IDLE) && s_axis_tvalid;

    // 输出拍: 一拍拍被下游接收
    wire out_beat = m_axis_tvalid && m_axis_tready;

    // FILL_DUMMY 填充完成: 当前拍输出后 byte_cnt 达到帧尾前位置
    //   (byte_cnt + 一拍字节 >= TAIL_POS)
    wire fill_done = (byte_cnt + BYTES_PER_BEAT >= TAIL_POS);

    // ========================================================================
    // 状态转移 (组合逻辑)
    // ========================================================================
    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE: begin
                if (sof)
                    state_next = S_INSERT_HEADER;
            end

            S_INSERT_HEADER: begin
                // 帧头被下游接收 → 进入转发阶段
                if (m_axis_tready)
                    state_next = S_FORWARD;
            end

            S_FORWARD: begin
                // ★ v2.0: 溢出 → 切全F填充 (丢弃本帧剩余)
                if (frame_overflowed)
                    state_next = S_FILL_DUMMY;
                // ★ v2.1: 像素最后一拍已接收 (s_axis_tlast) → 插帧尾
                //   修复双 tlast: 不再透传像素 tlast (SGDMA 按 tlast 收尾,
                //   desc.length 与 tlast 字节数不匹配 → 帧尾丢失)
                else if (s_axis_tvalid && s_axis_tready && s_axis_tlast)
                    state_next = S_INSERT_TAIL;
            end

            S_FILL_DUMMY: begin
                // 全F填充到帧尾前位置 → 插帧尾
                if (m_axis_tready && fill_done)
                    state_next = S_INSERT_TAIL;
            end

            S_INSERT_TAIL: begin
                // 帧尾被下游接收 → 回 IDLE 等下一帧
                if (m_axis_tready)
                    state_next = S_IDLE;
            end

            default: state_next = S_IDLE;
        endcase
    end

    // ========================================================================
    // 输出多路复用
    // ========================================================================
    assign m_axis_tdata  = (state == S_INSERT_HEADER) ? header              :
                           (state == S_INSERT_TAIL)   ? tail                :
                           (state == S_FILL_DUMMY)    ? {DATA_WIDTH{1'b1}}  :  // 全F
                           (buf_valid)                 ? buf_tdata          :
                                                         s_axis_tdata;

    assign m_axis_tkeep  = (state == S_INSERT_HEADER) ? {KEEP_WIDTH{1'b1}} :
                           (state == S_INSERT_TAIL)   ? {KEEP_WIDTH{1'b1}} :
                           (state == S_FILL_DUMMY)    ? {KEEP_WIDTH{1'b1}} :
                           (buf_valid)                 ? buf_tkeep          :
                                                         s_axis_tkeep;

    // ★ v2.1: 修复双 tlast — 只有帧尾拍 (INSERT_TAIL) 输出 tlast=1,
    //   像素最后一拍不再透传 tlast
    assign m_axis_tlast  = (state == S_INSERT_TAIL) ? 1'b1 : 1'b0;

    // m_axis_tvalid: 帧头/帧尾/全F 状态始终有效, 否则有数据就有效
    assign m_axis_tvalid = (state == S_INSERT_HEADER) ? 1'b1 :
                           (state == S_INSERT_TAIL)   ? 1'b1 :
                           (state == S_FILL_DUMMY)    ? 1'b1 :
                           (buf_valid || (s_axis_tvalid && state == S_FORWARD));

    // ========================================================================
    // s_axis_tready: 反压上游
    // ========================================================================
    // IDLE: 始终可接收 SOF
    // INSERT_HEADER: buf 满 → 不能收 (仅 1 拍, 可忽略)
    // FORWARD: buf 空且下游有空 → 可接收
    // FILL_DUMMY / INSERT_TAIL: 不读 FIFO (0)
    assign s_axis_tready = (state == S_IDLE) ||
                           (state == S_FORWARD && !buf_valid && m_axis_tready);

    // ========================================================================
    // 时序逻辑
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state            <= S_IDLE;
            buf_valid        <= 1'b0;
            buf_tdata        <= {DATA_WIDTH{1'b0}};
            buf_tkeep        <= {KEEP_WIDTH{1'b0}};
            buf_tlast        <= 1'b0;
            frame_overflowed <= 1'b0;
            frame_active     <= 1'b0;
            byte_cnt         <= {CNT_WIDTH{1'b0}};
        end else begin
            state <= state_next;

            // ----- byte_cnt 累加: 每输出一拍拍 +16 -----
            if (out_beat)
                byte_cnt <= byte_cnt + BYTES_PER_BEAT;

            // ----- SOF: 捕获首拍到 buf, 清溢出锁存, 拉高 frame_active, 清计数 -----
            if (sof) begin
                buf_tdata        <= s_axis_tdata;
                buf_tkeep        <= s_axis_tkeep;
                buf_tlast        <= s_axis_tlast;
                buf_valid        <= 1'b1;
                frame_overflowed <= 1'b0;
                frame_active     <= 1'b1;
                byte_cnt         <= {CNT_WIDTH{1'b0}};   // 覆盖累加 (IDLE 时 out_beat=0)
            end

            // ----- ★ 溢出锁存: overflow_in 拉高 → 本帧标记溢出 -----
            if (overflow_in)
                frame_overflowed <= 1'b1;

            // ----- FORWARD: 缓冲数据被取走 → 释放 buf -----
            if (state == S_FORWARD && buf_valid && out_beat) begin
                buf_valid <= 1'b0;
            end

            // ----- ★ 帧尾发出 → frame_id++ + frame_active 拉低 -----
            if (state == S_INSERT_TAIL && out_beat) begin
                frame_active <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
