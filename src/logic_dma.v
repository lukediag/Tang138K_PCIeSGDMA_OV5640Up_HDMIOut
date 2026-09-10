// ============================================================================
// logic_dma v5.3.0 — BAR2 寄存器 + 握手协议 (DDR3 bypass)
//
// ★ v5.1.0 Handshake 改动:
//   - Status 重定义: bit0=video_vs, [15:0]=frame_id (vsync 下降沿计数)
//   - Ctrl bit4=host_ready: Host 准备好接收下一帧
//   - stream_gate 输出: 供 cam_pclk 域 gate 使用
//   - frame_id: 由 video_vs 下降沿驱动 (不是 header_inserter tlast)
//
// ★ v5.2.0 改动 (环形 5 帧持续流):
//   - host_ready 纯锁存, host 每帧重配 SGDMA (FLAG_LAST 自动停)
//
// ★ v5.3.0 改动 (帧边界对齐开门):
//   - gate = host_ready 武装 + vsync↓ 触发: gate 只在帧边界开启
//   - 数据从帧头流入, 天然无半帧错位 (不再需要 stream_reset 对齐)
//
// 保留:
//   - H2C 描述符寄存器 (0x10/0x14/0x18/0x1C)
//   - stream_reset (0x08)
//
// 时钟域: tlp_clk (100MHz)
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module logic_dma #(
    parameter integer AXI_ADDR_WIDTH = 28,
    parameter integer AXI_LEN_WIDTH  = 29
) (
    input  wire         clk,
    input  wire         rst_n,

    // ---- BAR2 接口 (来自 Pcie_Sgdma_Top) ----
    input  wire         user_cs,
    input  wire [63:0]  user_address,
    input  wire         user_rw,            // 1=写, 0=读
    input  wire [31:0]  user_wr_data,
    output reg          user_rd_valid,
    output reg  [31:0]  user_rd_data,

    // ---- H2C 描述符 (→ SGDMA H2C 路径, 保留) ----
    output reg  [AXI_ADDR_WIDTH-1:0] m_axis_h2c_desc_addr,
    output reg  [ AXI_LEN_WIDTH-1:0] m_axis_h2c_desc_len,
    output reg                       m_axis_h2c_desc_valid,
    input  wire                      m_axis_h2c_desc_ready,
    input  wire [              63:0] h2c_overhead_reg,

    // ---- ★ v5.1.0: 视频同步信号 (tlp_clk 域, 已在 top.v 完成 CDC) ----
    input  wire         video_vs_tlp,

    // ---- ★ v5.1.0: 握手输出 ----
    output wire         stream_gate,        // = host_ready, 供 gate FSM
    output wire [15:0]  frame_id,           // 帧 ID (vsync 下降沿递增)
    output reg          stream_reset        // ★ 保留: C2H 数据流复位
);

    localparam integer USR_ADDR_WIDTH = 8;

    // =========================================================================
    // 寄存器地址
    // =========================================================================
    localparam [USR_ADDR_WIDTH-1:0] RegCtrl          = 8'h00;
    localparam [USR_ADDR_WIDTH-1:0] RegStatus        = 8'h04;
    localparam [USR_ADDR_WIDTH-1:0] RegReset         = 8'h08;  // ★ stream_reset
    localparam [USR_ADDR_WIDTH-1:0] RegVersion       = 8'h0C;  // ★ v1.0: FPGA 版本号 (只读)
    localparam [USR_ADDR_WIDTH-1:0] RegAddrDDRh2c    = 8'h10;
    localparam [USR_ADDR_WIDTH-1:0] RegLengDDRh2c    = 8'h14;
    localparam [USR_ADDR_WIDTH-1:0] RegOverheadh2cLo = 8'h18;
    localparam [USR_ADDR_WIDTH-1:0] RegOverheadh2cHi = 8'h1C;
    // 0x20-0x38: C2H/LAD 旧寄存器 — 读返回 0
    // 0x40-0x50: Video 旧寄存器 — 读返回 0

    // ★ v2.0: FPGA 版本号 (major=2, minor=0, 新帧头帧尾格式, 与 host HOST_VERSION 一致)
    localparam [31:0] FPGA_VERSION = 32'h00000200;

    // =========================================================================
    // ★ v2.0: 握手寄存器 (host_ready 锁存 + vsync↓ frame_id 计数)
    //   - gate 开门 FSM 移到 top.v 写侧 (cam_pclk 域), 这里只输出 host_ready
    // =========================================================================
    reg         host_ready;         // BAR2 Ctrl bit4 (host 武装, 任意时刻可写 1)
    reg [15:0]  frame_id_reg;       // vsync 下降沿递增
    reg         video_vs_d1;        // vsync 延迟 1 拍, 边沿检测

    assign stream_gate = host_ready;   // ★ v2.0: 直接输出 host_ready (gate FSM 在 top.v 写侧)
    assign frame_id    = frame_id_reg;

    // video_vs 下降沿 (1→0): 有效数据开始 → frame_id++
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_id_reg <= 16'd0;
            video_vs_d1  <= 1'b1;
        end else begin
            video_vs_d1 <= video_vs_tlp;
            if (video_vs_d1 && !video_vs_tlp) begin
                frame_id_reg <= frame_id_reg + 1'b1;
            end
        end
    end

    wire wr_en = user_cs && user_rw;
    wire rd_en = user_cs && !user_rw;
    wire [USR_ADDR_WIDTH-1:0] addr_reg = user_address[USR_ADDR_WIDTH-1:0];

    // =========================================================================
    // BAR2 写
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_h2c_desc_addr  <= {AXI_ADDR_WIDTH{1'b0}};
            m_axis_h2c_desc_len   <= {AXI_LEN_WIDTH{1'b0}};
            m_axis_h2c_desc_valid <= 1'b0;
            stream_reset          <= 1'b0;
            host_ready            <= 1'b0;
        end else begin
            // H2C 描述符握手完成 → 清除 valid
            if (m_axis_h2c_desc_valid && m_axis_h2c_desc_ready)
                m_axis_h2c_desc_valid <= 1'b0;

            if (wr_en) begin
                case (addr_reg)
                    RegCtrl: begin
                        // bit0: H2C start → 触发描述符发送
                        // bit1: H2C stop  → 清除 valid
                        if (user_wr_data[0])
                            m_axis_h2c_desc_valid <= 1'b1;
                        if (user_wr_data[1])
                            m_axis_h2c_desc_valid <= 1'b0;
                        // ★ v5.2.0: bit4 = host_ready (纯锁存: 写1开门, 写0关门)
                        host_ready <= user_wr_data[4];
                    end

                    RegAddrDDRh2c: begin
                        m_axis_h2c_desc_addr <= user_wr_data[AXI_ADDR_WIDTH-1:0];
                    end
                    RegLengDDRh2c: begin
                        m_axis_h2c_desc_len <= user_wr_data[AXI_LEN_WIDTH-1:0];
                    end

                    // ★ v3.0: stream_reset (写 1 = 复位 C2H 数据流, 写 0 = 释放)
                    RegReset: begin
                        stream_reset <= user_wr_data[0];
                    end

                    default: ;  // 其余寄存器写忽略
                endcase
            end
        end
    end

    // =========================================================================
    // BAR2 读
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            user_rd_valid <= 1'b0;
            user_rd_data  <= 32'd0;
        end else begin
            user_rd_valid <= 1'b0;

            if (rd_en) begin
                user_rd_valid <= 1'b1;
                case (addr_reg)
                    RegCtrl: begin
                        user_rd_data     <= 32'd0;
                        user_rd_data[4]  <= host_ready;
                    end
                    RegStatus: begin
                        user_rd_data       <= 32'd0;
                        user_rd_data[0]    <= video_vs_tlp;       // vsync 状态
                        user_rd_data[15:0] <= frame_id_reg;       // 帧 ID
                    end

                    RegAddrDDRh2c: begin
                        user_rd_data <= {{(32 - AXI_ADDR_WIDTH){1'b0}}, m_axis_h2c_desc_addr};
                    end
                    RegLengDDRh2c: begin
                        user_rd_data <= {{(32 - AXI_LEN_WIDTH){1'b0}}, m_axis_h2c_desc_len};
                    end

                    RegOverheadh2cLo: user_rd_data <= h2c_overhead_reg[31:0];
                    RegOverheadh2cHi: user_rd_data <= h2c_overhead_reg[63:32];

                    // ★ v3.0: 读回 stream_reset 状态
                    RegReset: user_rd_data <= {31'd0, stream_reset};

                    // ★ v1.0: FPGA 版本号 (只读)
                    RegVersion: user_rd_data <= FPGA_VERSION;

                    default: user_rd_data <= 32'd0;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
