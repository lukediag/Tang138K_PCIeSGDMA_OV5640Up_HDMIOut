// ============================================================================
// axis_width_down: 256-bit → 128-bit AXI-Stream 宽度转换
//
// 数据流:
//   SGDMA(256-bit) → axis_width_down → axis_async_fifo(128-bit) → DDR
//
// 转换规则:
//   1拍 256-bit → 2拍 128-bit（高128先出，低128后出）
//   如果高128的 tkeep 全为0 → 只出1拍（低128），跳过高半拍
//
// 连续包反压:
//   状态机只有 IDLE 状态才拉 s_axis_tready，其他状态 tready=0
//   → SGDMA 自动等待，数据不会丢失
//
// 时钟域: 全部在 tlp_clk 域（和 SGDMA 同域）
// ============================================================================

module axis_width_down (
    input  wire         clk,
    input  wire         rst,            // 高有效复位

    // === 256-bit 输入 (从 SGDMA H2C) ===
    input  wire [255:0] s_axis_tdata,
    input  wire [31:0]  s_axis_tkeep,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire         s_axis_tlast,

    // === 128-bit 输出 (到 async FIFO) ===
    output reg  [127:0] m_axis_tdata,
    output reg  [15:0]  m_axis_tkeep,
    output reg          m_axis_tvalid,
    input  wire         m_axis_tready,
    output reg          m_axis_tlast
);

    // ========================================================================
    // 状态机: IDLE → OUT_UPPER → OUT_LOWER → IDLE (或 IDLE → OUT_LOWER → IDLE)
    // ========================================================================
    localparam IDLE      = 2'd0;   // 等待输入，tready=1
    localparam OUT_UPPER = 2'd1;   // 正在输出高128位
    localparam OUT_LOWER = 2'd2;   // 正在输出低128位（或唯一拍）

    reg [1:0] state = IDLE;

    // 输入数据锁存 —— 只在 IDLE 态捕获一拍，整个输出周期保持
    reg [255:0] buf_tdata;
    reg [31:0]  buf_tkeep;
    reg         buf_tlast;

    // ------------------------------------------------------------------------
    // s_axis_tready: 只在 IDLE 态才接收新数据
    // 任何非 IDLE 态都拉低 → SGDMA 自动等待，不会丢数据
    // ------------------------------------------------------------------------
    assign s_axis_tready = (state == IDLE);

    // ------------------------------------------------------------------------
    // 组合逻辑输出 —— 直接从 buffer 取数，无延迟
    // ------------------------------------------------------------------------
    always @(*) begin
        case (state)
            OUT_UPPER: begin
                m_axis_tdata  = buf_tdata[255:128];
                m_axis_tkeep  = buf_tkeep[31:16];
                m_axis_tvalid = 1'b1;
                m_axis_tlast  = 1'b0;          // 高半拍永远不带 tlast
            end
            OUT_LOWER: begin
                m_axis_tdata  = buf_tdata[127:0];
                m_axis_tkeep  = buf_tkeep[15:0];
                m_axis_tvalid = 1'b1;
                m_axis_tlast  = buf_tlast;     // tlast 落在最后一个低半拍
            end
            default: begin  // IDLE
                m_axis_tdata  = 128'd0;
                m_axis_tkeep  = 16'd0;
                m_axis_tvalid = 1'b0;
                m_axis_tlast  = 1'b0;
            end
        endcase
    end

    // ------------------------------------------------------------------------
    // 时序逻辑: 状态转移 + 输入锁存
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
        end else begin
            case (state)
                // ============================================================
                // IDLE: 等待上游发数据
                // ============================================================
                IDLE: begin
                    if (s_axis_tvalid && s_axis_tready) begin
                        // 锁存一整拍 256-bit 数据
                        buf_tdata <= s_axis_tdata;
                        buf_tkeep <= s_axis_tkeep;
                        buf_tlast <= s_axis_tlast;
                        // 判断是否需要拆成两拍
                        if (|s_axis_tkeep[31:16]) begin
                            state <= OUT_UPPER;   // 高128有有效字节 → 两拍
                        end else begin
                            state <= OUT_LOWER;   // 高128全0 → 只出一拍
                        end
                    end
                end

                // ============================================================
                // OUT_UPPER: 等高半拍被下游收走 → 转入低半拍
                // ============================================================
                OUT_UPPER: begin
                    if (m_axis_tready)
                        state <= OUT_LOWER;
                end

                // ============================================================
                // OUT_LOWER: 等低半拍被下游收走 → 回到 IDLE
                //           回到 IDLE 后 tready 拉高，SGDMA 可以发下一拍
                // ============================================================
                OUT_LOWER: begin
                    if (m_axis_tready)
                        state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
