// ============================================================================
// axis_width_up: 128-bit → 256-bit AXI-Stream 宽度转换
//
// 数据流:
//   DDR → async_fifo(128-bit) → axis_width_up(128→256) → SGDMA C2H
//
// 转换规则:
//   2拍 128-bit → 1拍 256-bit（先到的是低128，后到的是高128）
//   ★ v1.5 swap 修复: 先到的放低 128bit — SGDMA 写内存时低 128bit 先写,
//     避免每 8 像素前 4 后 4 翻转 (原 {lo_tdata, s_axis_tdata} 先到放高位导致 swap)
//   如果单拍带了 tlast → 高位补0，直接输出（不需要等第二拍）
//
// 连续包反压:
//   lo_valid=1 表示已缓存低半拍，等上半拍。
//   lo_valid=1 且 m_axis_tvalid=1（等待上次输出被收）时 tready=0 → 反压上游
//
// 时钟域: 全部在 tlp_clk 域（和 SGDMA 同域）
// ============================================================================

module axis_width_up (
    input  wire         clk,
    input  wire         rst,            // 高有效复位

    // === 128-bit 输入 (从 async FIFO C2H 来) ===
    input  wire [127:0] s_axis_tdata,
    input  wire [15:0]  s_axis_tkeep,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire         s_axis_tlast,

    // === 256-bit 输出 (到 SGDMA C2H) ===
    output reg  [255:0] m_axis_tdata,
    output reg  [31:0]  m_axis_tkeep,
    output reg          m_axis_tvalid,
    input  wire         m_axis_tready,
    output reg          m_axis_tlast
);

    // 低半拍缓存 —— 等上半拍到了拼成 256-bit 一起出
    reg         lo_valid;       // 1 = 已缓存低半拍，等上半拍
    reg [127:0] lo_tdata;
    reg [15:0]  lo_tkeep;

    // ------------------------------------------------------------------------
    // s_axis_tready: 能接收新数据的条件
    //   条件1: lo_valid=0 → 缓存空，随时可收（新低半拍）
    //   条件2: lo_valid=1 且 m_axis_tvalid=0 → 缓存了低半拍，输出空闲，
    //          收到上半拍立刻拼装输出，不冲突
    //   !lo_valid || !m_axis_tvalid 正好覆盖两种合法场景
    // ------------------------------------------------------------------------
    assign s_axis_tready = !lo_valid || !m_axis_tvalid;

    // ------------------------------------------------------------------------
    // 时序逻辑
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            lo_valid     <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
        end else begin
            // ----- 下游收走了上次的 256-bit 输出 -----
            if (m_axis_tvalid && m_axis_tready)
                m_axis_tvalid <= 1'b0;

            // ----- 接收新的 128-bit 输入 -----
            if (s_axis_tvalid && s_axis_tready) begin

                if (s_axis_tlast) begin
                    // ========================================================
                    // tlast=1: 这是包尾拍，不等的第二拍
                    // ========================================================
                    if (lo_valid) begin
                        // 缓存里有低半拍 + 当前是带 tlast 的高半拍
                        // → 拼装成完整的最后一拍 256-bit
                        // ★ v1.5 swap 修复: 先到的放低 128bit (SGDMA 低128先写)
                        m_axis_tdata  <= {s_axis_tdata, lo_tdata};
                        m_axis_tkeep  <= {s_axis_tkeep, lo_tkeep};
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= 1'b1;
                        lo_valid      <= 1'b0;
                    end else begin
                        // 缓存空 + 当前拍 tlast → 奇数长度的包
                        // → 低128放数据 (SGDMA 低128先写)
                        m_axis_tdata  <= {s_axis_tdata, 128'd0};
                        m_axis_tkeep  <= {s_axis_tkeep, 16'd0};
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= 1'b1;
                        // lo_valid 已经是 0
                    end

                end else begin
                    // ========================================================
                    // tlast=0: 普通数据拍
                    // ========================================================
                    if (lo_valid) begin
                        // 缓存了低半拍 + 当前是上半拍 → 拼装 256-bit 输出
                        // ★ v1.5 swap 修复: 先到的放低 128bit (SGDMA 低128先写)
                        m_axis_tdata  <= {s_axis_tdata, lo_tdata};
                        m_axis_tkeep  <= {s_axis_tkeep, lo_tkeep};
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= 1'b0;
                        lo_valid      <= 1'b0;
                    end else begin
                        // 缓存空 → 当前拍作为低半拍缓存，等上半拍
                        lo_tdata  <= s_axis_tdata;
                        lo_tkeep  <= s_axis_tkeep;
                        lo_valid  <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
