// ============================================================================
// SGDMA_OV5640 v1.1 — OV5640 摄像头采集 + PCIe SGDMA C2H (RGB565 直传)
// ============================================================================
// 功能: OV5640 摄像头 (DVP RGB565 直传) → SGDMA C2H → PCIe → ARM64 Host
// 板卡: Sipeed Tang Mega 138K Pro (GW5AT-138B)
//
// === 核心改动 (v1.0 vs 2v2 彩条版) ===========================================
//   ★ 采集源: video_display 彩条 → OV5640 真实摄像头
//   ★ 时钟域: pixel_clk(75M) → cam_pclk(48M), packer+gate+CDC写端全切到 cam_pclk
//   ★ vsync 源: HDMI video_vs → cmos_frame_vsync (摄像头场同步)
//   ★ HDMI 本地显示保留 (彩条监视器), 与采集链路相互独立
//   ★ 版本号: FPGA/host = 1.1 (0x00000101, RGB565 直传)
//
// === 数据流 ==================================================================
//   OV5640(8bit) → ov5640_dri → RGB565 (16bit 直传, 不经 888 转换)
//     → video_axis_packer(128b, 8像素/拍) → CDC FIFO(cam_pclk→tlp_clk)
//       → frame_header_inserter → width_up(128→256) → SGDMA C2H → Host
//
// === 时钟架构 (4 域) =========================================================
//   tlp_clk   (100MHz) — PCIe SGDMA, BAR2 logic_dma, C2H 数据路径
//   pixel_clk  (75MHz) — HDMI 本地显示 (video_driver/video_display/DVI_TX)
//   pixel_clk_5x (375MHz) — TMDS serializer
//   cam_pclk   (~48MHz) — OV5640 采集 + packer + gate + CDC FIFO 写端
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module top (
    // ========================================================================
    // 时钟 & 复位 输入
    // ========================================================================
    input  wire         sys_clkin,        // 25MHz 系统晶振
    input  wire         pcie_rst_n,       // PCIe 复位 (低有效)
    input  wire         hard_rst_n,       // 硬件复位 (低有效)

    // ========================================================================
    // LED 输出
    // ========================================================================
    output wire  [7:0]  LED,              // [0]=心跳50M [1]=心跳100M [2]=心跳50M
                                          // [3]=~pcie_rst [4]=pcie_linkup
                                          // [5]=h2c_run [6]=c2h_run [7]=pcie_linkup

    // ========================================================================
    // HDMI / DVI 输出 (TMDS 差分对)
    // ========================================================================
    output wire         tmds_clk_p,       // TMDS 时钟 P
    output wire         tmds_clk_n,       // TMDS 时钟 N
    output wire  [2:0]  tmds_data_p,      // TMDS 数据 P (R/G/B)
    output wire  [2:0]  tmds_data_n,      // TMDS 数据 N (R/G/B)

    // ========================================================================
    // OV5640 摄像头接口 (DVP + SCCB)
    // ========================================================================
    input  wire         cam_pclk,         // OV5640 像素时钟 (输入, ~48MHz)
    input  wire         cam_vsync,        // OV5640 场同步 (低有效同步)
    input  wire         cam_href,         // OV5640 行同步 (高有效)
    input  wire  [7:0]  cam_data,         // OV5640 DVP 8-bit 数据
    inout  wire         cam_sda,          // SCCB SDA
    output wire         cam_scl,          // SCCB SCL
    output wire         cam_rst_n,        // OV5640 复位 (低有效)
    output wire         cam_pwdn          // OV5640 电源休眠 (0=正常)
);


    // ========================================================================
    // 第 1 节: 参数定义
    // ========================================================================

    // ----- 复位时序参数 -----
    localparam integer PCIE_DLY     = 8;    // PCIe 启动等待周期数
    localparam integer PERST_DLY    = 25;   // PCIe 复位释放延迟
    localparam integer SYS_RST_DLY  = 20;   // 系统复位释放延迟

    // ----- AXI-Stream 参数 -----
    localparam integer AXI_DATA_WIDTH  = 128;                    // 数据位宽 (bit)
    localparam integer AXI_STRB_WIDTH  = AXI_DATA_WIDTH / 8;     // 字节使能位宽 = 16
    localparam integer AXI_ADDR_WIDTH  = 28;                     // 地址位宽 (logic_dma)
    localparam integer AXI_LEN_WIDTH   = AXI_ADDR_WIDTH + 1;     // 长度位宽 = 29
    localparam integer VIDEO_FIFO_DEPTH = 65536;                 // CDC FIFO 深度 (1 MB, v1.5.1 覆盖握手延迟)


    // ========================================================================
    // 第 2 节: 时钟 & 复位生成
    // ========================================================================

    // ----- 系统 PLL: 25MHz → 50/200/400 MHz -----
    wire        sys_clk;           // 200MHz, 系统主时钟
    wire        pll_50m_clk;       // 50MHz, heartbeat
    wire        pll_200m_clk;      // 200MHz, PLL 直出
    wire        pll_400m_clk;      // 400MHz, 未使用 (DDR3 已删除)
    wire        pll_lock;          // PLL 锁定标志
    wire        pll_stop = 1'b0;   // 禁用 400MHz 输出 (无 DDR3)

    Gowin_PLL u_pll (
        .clkin   (sys_clkin),
        .init_clk(sys_clkin),       // 初始配置时钟 = 晶振
        .clkout0 (pll_50m_clk),     // 50MHz
        .clkout1 (pll_200m_clk),    // 200MHz
        .clkout2 (pll_400m_clk),    // 400MHz (禁用)
        .enclk0  (1'b1),            // 50M 始终使能
        .enclk1  (1'b1),            // 200M 始终使能
        .enclk2  (pll_stop),        // 400M 禁用
        .lock    (pll_lock),
        .reset   (~hard_rst_n)      // 硬件复位时复位 PLL
    );
    assign sys_clk = pll_200m_clk;

    // ----- CLKDIV: 200MHz ÷ 2 = 100MHz -----
    // 100MHz 用于: PCIe TLP 接口 + C2H 数据路径 + BAR2
    /* synthesis syn_keep = 1 */ wire div_clk;
    /* synthesis syn_keep = 1 */ wire tlp_clk;     // = 100MHz, PCIe SGDMA 时钟

    CLKDIV #(
        .DIV_MODE("2")
    ) uut_div2 (
        div_clk,
        'b0,
        sys_clk,
        'b1
    );
    assign tlp_clk = div_clk;

    // ----- 系统复位链 -----
    reg  [26:0] pcie_st_cnt  = 0;
    reg  [26:0] run_cnt      = 0;
    reg  [26:0] perst_cnt    = 0;
    reg  [SYS_RST_DLY:0] sys_rst_cnt = 0;

    wire        rst_n  = hard_rst_n & pcie_rst_n;

    // 系统复位延迟释放 (等待 PLL 稳定)
    always @(posedge tlp_clk or negedge rst_n)
        if (!rst_n)
            sys_rst_cnt <= 0;
        else if (!sys_rst_cnt[SYS_RST_DLY])
            sys_rst_cnt <= sys_rst_cnt + 1'd1;

    wire sys_rst_n = sys_rst_cnt[SYS_RST_DLY];   // 延迟后的系统复位

    // PCIe 复位延迟
    always @(posedge tlp_clk or negedge sys_rst_n)
        if (!sys_rst_n)
            perst_cnt <= 0;
        else if (!perst_cnt[PERST_DLY])
            perst_cnt <= perst_cnt + 1'd1;

    // PCIe 启动延迟 (等 PCIe PHY 稳定)
    reg pcie_start;
    always @(posedge tlp_clk or negedge sys_rst_n)
        if (!sys_rst_n)
            pcie_st_cnt <= 0;
        else if (!pcie_start)
            pcie_st_cnt <= pcie_st_cnt + 1'd1;

    always @(*) pcie_start = pcie_st_cnt[PCIE_DLY] ? 1'b1 : 1'b0;

    // 运行计数器 (用于 LED 闪烁)
    always @(posedge tlp_clk or negedge rst_n)
        if (!rst_n)
            run_cnt <= 0;
        else
            run_cnt <= run_cnt + 1'd1;

    // PCIe 链路状态 (打一拍同步)
    wire        pcie_linkup;
    reg         pcie_linkup_r = 0;
    /* synthesis syn_keep = 1 */
    always @(posedge tlp_clk) pcie_linkup_r <= pcie_linkup;


    // ========================================================================
    // 第 3 节: PCIe 核心 (SerDes PHY + Controller)
    // ========================================================================
    // 与 v5.0.2 完全相同, 无改动

    // ----- PCIe TLP 接口信号 -----
    wire [  4:0] pcie_ltssm;
    wire         pcie_tl_rx_sop;
    wire         pcie_tl_rx_eop;
    wire [255:0] pcie_tl_rx_data;
    wire [  7:0] pcie_tl_rx_valid;
    wire [  5:0] pcie_tl_rx_bardec;
    wire [  7:0] pcie_tl_rx_err;
    wire         pcie_tl_rx_wait;
    wire         pcie_tl_rx_masknp;
    wire         pcie_tl_tx_sop;
    wire         pcie_tl_tx_eop;
    wire [255:0] pcie_tl_tx_data;
    wire [  7:0] pcie_tl_tx_valid;
    wire         pcie_tl_tx_wait;
    // DRP
    wire         pcie_tl_drp_clk;
    wire [ 23:0] pcie_tl_drp_addr;
    wire         pcie_tl_drp_ready;
    wire [  7:0] pcie_tl_drp_strb;
    wire         pcie_tl_drp_resp;
    wire         pcie_tl_drp_wr;
    wire [ 31:0] pcie_tl_drp_wrdata;
    wire         pcie_tl_drp_rd;
    wire [ 31:0] pcie_tl_drp_rddata;
    wire         pcie_tl_drp_rd_valid;
    wire         pcie_tl_int_req;
    wire         pcie_tl_int_ack;
    wire         pcie_tl_int_status;
    wire [  4:0] pcie_tl_int_msinum;
    wire [ 12:0] pcie_tl_cfg_busdev;

    SerDes_Top u_pcie_ip (
        .PCIE_Controller_Top_pcie_rstn_i       (rst_n),
        .PCIE_Controller_Top_pcie_tl_clk_i     (tlp_clk),
        .PCIE_Controller_Top_pcie_linkup_o     (pcie_linkup),
        .PCIE_Controller_Top_pcie_ltssm_o      (pcie_ltssm),
        .PCIE_Controller_Top_pcie_tl_rx_sop_o  (pcie_tl_rx_sop),
        .PCIE_Controller_Top_pcie_tl_rx_eop_o  (pcie_tl_rx_eop),
        .PCIE_Controller_Top_pcie_tl_rx_data_o (pcie_tl_rx_data),
        .PCIE_Controller_Top_pcie_tl_rx_valid_o(pcie_tl_rx_valid),
        .PCIE_Controller_Top_pcie_tl_rx_bardec_o(pcie_tl_rx_bardec),
        .PCIE_Controller_Top_pcie_tl_rx_err_o  (pcie_tl_rx_err),
        .PCIE_Controller_Top_pcie_tl_rx_wait_i (pcie_tl_rx_wait),
        .PCIE_Controller_Top_pcie_tl_rx_masknp_i(pcie_tl_rx_masknp),
        .PCIE_Controller_Top_pcie_tl_tx_sop_i  (pcie_tl_tx_sop),
        .PCIE_Controller_Top_pcie_tl_tx_eop_i  (pcie_tl_tx_eop),
        .PCIE_Controller_Top_pcie_tl_tx_data_i (pcie_tl_tx_data),
        .PCIE_Controller_Top_pcie_tl_tx_valid_i(pcie_tl_tx_valid),
        .PCIE_Controller_Top_pcie_tl_tx_wait_o (pcie_tl_tx_wait),
        .PCIE_Controller_Top_pcie_tl_drp_clk_o     (pcie_tl_drp_clk),
        .PCIE_Controller_Top_pcie_tl_drp_addr_i    (pcie_tl_drp_addr),
        .PCIE_Controller_Top_pcie_tl_drp_ready_o   (pcie_tl_drp_ready),
        .PCIE_Controller_Top_pcie_tl_drp_resp_o    (pcie_tl_drp_resp),
        .PCIE_Controller_Top_pcie_tl_drp_strb_i    (pcie_tl_drp_strb),
        .PCIE_Controller_Top_pcie_tl_drp_wr_i      (pcie_tl_drp_wr),
        .PCIE_Controller_Top_pcie_tl_drp_wrdata_i  (pcie_tl_drp_wrdata),
        .PCIE_Controller_Top_pcie_tl_drp_rd_i      (pcie_tl_drp_rd),
        .PCIE_Controller_Top_pcie_tl_drp_rddata_o  (pcie_tl_drp_rddata),
        .PCIE_Controller_Top_pcie_tl_drp_rd_valid_o(pcie_tl_drp_rd_valid),
        .PCIE_Controller_Top_pcie_tl_int_req_i     (pcie_tl_int_req),
        .PCIE_Controller_Top_pcie_tl_int_ack_o     (pcie_tl_int_ack),
        .PCIE_Controller_Top_pcie_tl_int_status_i  (pcie_tl_int_status),
        .PCIE_Controller_Top_pcie_tl_int_msinum_i  (pcie_tl_int_msinum),
        .PCIE_Controller_Top_pcie_tl_cfg_busdev_o  (pcie_tl_cfg_busdev),
        .por_n_i                                   (pcie_rst_n)
    );


    // ========================================================================
    // 第 4 节: PCIe SGDMA 引擎 (Gowin IP)
    // ========================================================================
    // H2C 通道: ★ 保留 (供未来控制通道使用)
    // C2H 通道: 接收 256-bit AXI-Stream (来自视频数据路径)

    // ----- H2C AXI-Stream (保留, 256-bit) -----
    wire         axis_h2c_tready;      // 由 width_down 反压控制
    wire         axis_h2c_tvalid;
    wire [255:0] axis_h2c_tdata;
    wire         axis_h2c_tlast;
    wire [ 31:0] axis_h2c_tkeep;
    wire [ 63:0] h2c_overhead;
    wire         h2c_run;

    // ----- H2C 128-bit 中间信号 (tlp_clk 域, width_down 输出) -----
    wire                      h2c_128_tready;
    wire                      h2c_128_tvalid;
    wire [AXI_DATA_WIDTH-1:0] h2c_128_tdata;    // 128-bit
    wire                      h2c_128_tlast;
    wire [AXI_STRB_WIDTH-1:0] h2c_128_tkeep;

    // ----- C2H AXI-Stream (FPGA → Host, 256-bit) -----
    wire         axis_c2h_tready;
    wire         axis_c2h_tvalid;
    wire [255:0] axis_c2h_tdata;
    wire         axis_c2h_tlast;
    wire [ 31:0] axis_c2h_tkeep;
    wire         c2h_run;

    // ----- BAR2 用户接口 (Host ↔ FPGA 寄存器) -----
    wire         user_cs;
    wire [ 63:0] user_address;
    wire         user_rw;
    wire [ 31:0] user_wr_data;
    wire         user_rd_valid;
    wire [ 31:0] user_rd_data;

    Pcie_Sgdma_Top u_pcie_sgdma (
        .pcie_rstn            (rst_n),
        .clk                  (tlp_clk),
        // PCIe TLP
        .pcie_tl_rx_sop       (pcie_tl_rx_sop),
        .pcie_tl_rx_eop       (pcie_tl_rx_eop),
        .pcie_tl_rx_data      (pcie_tl_rx_data),
        .pcie_tl_rx_valid     (pcie_tl_rx_valid),
        .pcie_tl_rx_bardec    (pcie_tl_rx_bardec),
        .pcie_tl_rx_err       (pcie_tl_rx_err),
        .pcie_tl_rx_wait      (pcie_tl_rx_wait),
        .pcie_tl_rx_masknp    (pcie_tl_rx_masknp),
        .pcie_tl_tx_sop       (pcie_tl_tx_sop),
        .pcie_tl_tx_eop       (pcie_tl_tx_eop),
        .pcie_tl_tx_data      (pcie_tl_tx_data),
        .pcie_tl_tx_valid     (pcie_tl_tx_valid),
        .pcie_tl_tx_wait      (pcie_tl_tx_wait),
        .pcie_tl_int_status   (pcie_tl_int_status),
        .pcie_tl_int_req      (pcie_tl_int_req),
        .pcie_tl_int_msinum   (pcie_tl_int_msinum),
        .pcie_tl_int_ack      (pcie_tl_int_ack),
        .pcie_tl_drp_clk      (pcie_tl_drp_clk),
        .pcie_tl_drp_addr     (pcie_tl_drp_addr),
        .pcie_tl_drp_wr       (pcie_tl_drp_wr),
        .pcie_tl_drp_wrdata   (pcie_tl_drp_wrdata),
        .pcie_tl_drp_strb     (pcie_tl_drp_strb),
        .pcie_tl_drp_rd       (pcie_tl_drp_rd),
        .pcie_tl_drp_ready    (pcie_tl_drp_ready),
        .pcie_tl_drp_rd_valid (pcie_tl_drp_rd_valid),
        .pcie_tl_drp_rddata   (pcie_tl_drp_rddata),
        .pcie_tl_drp_resp     (pcie_tl_drp_resp),
        .pcie_ltssm           (pcie_ltssm),
        .pcie_linkup          (pcie_linkup),
        .pcie_tl_cfg_busdev   (pcie_tl_cfg_busdev),
        // H2C: 不使用
        .m_axis_h2c_tready    (axis_h2c_tready),
        .m_axis_h2c_tvalid    (axis_h2c_tvalid),
        .m_axis_h2c_tdata     (axis_h2c_tdata),
        .m_axis_h2c_tlast     (axis_h2c_tlast),
        .m_axis_h2c_tkeep     (axis_h2c_tkeep),
        .h2c_overhead         (h2c_overhead),
        // C2H: 来自视频路径
        .s_axis_c2h_tready    (axis_c2h_tready),
        .s_axis_c2h_tvalid    (axis_c2h_tvalid),
        .s_axis_c2h_tlast     (axis_c2h_tlast),
        .s_axis_c2h_tdata     (axis_c2h_tdata),
        .s_axis_c2h_tkeep     (axis_c2h_tkeep),
        .c2h_overhead_valid   (1'b0),
        .c2h_overhead_data    (64'd0),
        // BAR2
        .user_cs              (user_cs),
        .user_address         (user_address),
        .user_rw              (user_rw),
        .user_wr_data         (user_wr_data),
        .user_rd_valid        (user_rd_valid),
        .user_rd_data         (user_rd_data),
        // 运行状态
        .h2c_run              (h2c_run),
        .c2h_run              (c2h_run)
    );


    // ========================================================================
    // 第 5 节: BAR2 控制逻辑 + ★ v2.0 握手协议
    // ========================================================================
    // BAR2 寄存器:
    //   0x00 Ctrl     — bit0=H2C start, bit1=H2C stop, ★ bit4=host_ready
    //   0x04 Status   — ★ bit0=video_vs, [15:0]=frame_id (vsync下降沿计数)
    //   0x08 Reset    — bit0=stream_reset
    //   0x10-0x1C     — H2C 保留
    //
    // 握手协议 (v2.0 简化):
    //   1. FPGA: vsync 高期间清 FIFO; vsync↓ → frame_id++ (logic_dma 内部)
    //   2. Host: 先 setup desc + start SGDMA (提前就位等数据), 再写 host_ready=1
    //   3. FPGA: host_ready=1 且 vsync↓ → gate 打开 (永久开, FIFO 刚被清)
    //   4. SGDMA 每帧 FLAG_LAST 自动停, host 搬完立即重配+重启下一帧
    //   5. Host 退出 → 写 host_ready=0 → gate 直接关

    // ----- 视频同步 CDC: cam_pclk → tlp_clk (摄像头 vsync 驱动 frame_id) -----
    reg  video_vs_tlp_d1, video_vs_tlp_d2;
    reg  video_de_tlp_d1,  video_de_tlp_d2;
    always @(posedge tlp_clk or negedge rst_n) begin
        if (!rst_n) begin
            video_vs_tlp_d1 <= 1'b1;  // 默认 vsync=1 (无消隐)
            video_vs_tlp_d2 <= 1'b1;
            video_de_tlp_d1  <= 1'b0;
            video_de_tlp_d2  <= 1'b0;
        end else begin
            video_vs_tlp_d1 <= cam_vs;         // from cam_pclk domain
            video_vs_tlp_d2 <= video_vs_tlp_d1;
            video_de_tlp_d1  <= cam_de;
            video_de_tlp_d2  <= video_de_tlp_d1;
        end
    end
    wire video_vs_tlp = video_vs_tlp_d2;

    // ----- H2C 描述符 (logic_dma → H2C 路径) -----
    wire [AXI_ADDR_WIDTH-1:0] axis_h2c_desc_addr;
    wire [ AXI_LEN_WIDTH-1:0] axis_h2c_desc_len;
    wire                      axis_h2c_desc_valid;
    wire                      axis_h2c_desc_ready;

    // H2C overhead 锁存
    reg [63:0] h2c_overhead_reg;

    // ★ v1.0 握手信号
    wire        stream_gate;       // tlp_clk 域: = host_ready
    wire [15:0] frame_id;          // tlp_clk 域: vsync 下降沿计数
    wire        stream_reset;      // tlp_clk 域: CDC FIFO + header 复位

    // ★ stream_gate CDC: tlp_clk → cam_pclk
    reg  stream_gate_cam_d1, stream_gate_cam_d2;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            stream_gate_cam_d1 <= 1'b0;
            stream_gate_cam_d2 <= 1'b0;
        end else begin
            stream_gate_cam_d1 <= stream_gate;
            stream_gate_cam_d2 <= stream_gate_cam_d1;
        end
    end
    wire host_ready_cam = stream_gate_cam_d2;

    // stream_reset CDC 到 cam_pclk (保留, CDC FIFO 写端复位)
    reg  stream_reset_cam_d1, stream_reset_cam_d2;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            stream_reset_cam_d1 <= 1'b0;
            stream_reset_cam_d2 <= 1'b0;
        end else begin
            stream_reset_cam_d1 <= stream_reset;
            stream_reset_cam_d2 <= stream_reset_cam_d1;
        end
    end
    wire stream_reset_cam = stream_reset_cam_d2;

    logic_dma #(
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_LEN_WIDTH (AXI_LEN_WIDTH)
    ) u_logic_dma (
        .clk                    (tlp_clk),
        .rst_n                  (rst_n),
        .user_cs                (user_cs),
        .user_address           (user_address),
        .user_rw                (user_rw),
        .user_wr_data           (user_wr_data),
        .user_rd_valid          (user_rd_valid),
        .user_rd_data           (user_rd_data),
        .m_axis_h2c_desc_addr   (axis_h2c_desc_addr),
        .m_axis_h2c_desc_len    (axis_h2c_desc_len),
        .m_axis_h2c_desc_valid  (axis_h2c_desc_valid),
        .m_axis_h2c_desc_ready  (axis_h2c_desc_ready),
        .h2c_overhead_reg       (h2c_overhead_reg),
        .video_vs_tlp           (video_vs_tlp),
        .stream_gate            (stream_gate),
        .frame_id               (frame_id),
        .stream_reset           (stream_reset)
    );


    // ========================================================================
    // 第 6 节: C2H 直接数据路径 (★ v5.0.3 核心改动)
    // ========================================================================
    // 数据流:
    //   OV5640(24b RGB888) → video_axis_packer(128b AXI-S)
    //     → async_fifo(cam_pclk→tlp_clk, 65536深)
    //       → frame_header_inserter(128b, 每帧首加 magic header + 帧尾)
    //         → axis_width_up(128→256)
    //           → SGDMA C2H(256b) → PCIe → Host
    //
    // 反压链路 (端到端):
    //   SGDMA C2H tready=0 → width_up tready=0 → header_inserter tready=0
    //     → CDC FIFO tready=0 → packer can_output=0 → frame_drop=1 → 丢整帧

    // ----- 6.1 HDMI 像素数据 (pixel_clk 域, 仅本地显示) -----
    wire [10:0] pixel_xpos_w;
    wire [10:0] pixel_ypos_w;
    wire [23:0] pixel_data_w;
    wire        video_hs;
    wire        video_vs;
    wire        video_de;
    wire [23:0] video_rgb;
    // ★ v1.1: 删除 video_pipe_stall — 摄像头不能 stall, 反压改为丢整帧

    // ----- 6.1b OV5640 摄像头数据 (cam_pclk 域, SGDMA 采集源) -----
    wire        cam_de;        // 像素有效 (= cmos_frame_valid)
    wire        cam_vs;        // 场同步   (= cmos_frame_vsync)
    wire [15:0] cam_rgb565;    // RGB565 像素数据 (直传, 不经 888 转换)

    // ----- 6.2 packer 输出: 128-bit AXI-Stream (cam_pclk 域) -----
    wire [127:0] video_packed_tdata;
    wire [ 15:0] video_packed_tkeep;
    wire         video_packed_tvalid;
    wire         video_packed_tready;
    wire         video_packed_tlast;

    // ----- 6.3 CDC FIFO 输出: 128-bit AXI-Stream (tlp_clk 域) -----
    wire [127:0] video_tlp_tdata;
    wire [ 15:0] video_tlp_tkeep;
    wire         video_tlp_tvalid;
    wire         video_tlp_tready;
    wire         video_tlp_tlast;

    // ----- 6.4 header_inserter 输出: 128-bit AXI-Stream (tlp_clk 域) -----
    wire [127:0] video_hdr_tdata;
    wire [ 15:0] video_hdr_tkeep;
    wire         video_hdr_tvalid;
    wire         video_hdr_tready;
    wire         video_hdr_tlast;

    // ----- 6.5 width_up 输出: 256-bit AXI-Stream (tlp_clk 域) -----
    wire [255:0] video_256_tdata;
    wire [ 31:0] video_256_tkeep;
    wire         video_256_tvalid;
    wire         video_256_tready;
    wire         video_256_tlast;

    // ----- ★ v2.0: 溢出检测 + gate FSM 信号 -----
    wire [16:0]  fifo_wr_depth;      // CDC FIFO 写侧水位 ($clog2(65536)+1 = 17bit)
    wire         frame_active;       // header_inserter 本帧进行中 (tlp_clk 域)

    // ====================================================================
    // 6.0 OV5640 摄像头采集 (★ v1.0: 彩条 → 真实摄像头)
    // ====================================================================
    // 数据流: OV5640(DVP 8bit) → ov5640_dri(I2C 配置 + 采集)
    //           → cmos_frame_data[15:0] (RGB565, cam_pclk 域)
    //             → video_axis_packer (16bit 直传, 不经 888 转换) → CDC FIFO → SGDMA
    // 分辨率: 1280×720 RGB565 (2B/像素, 1.84MB/帧; PCLK 72MHz → ~28.6fps)
    wire        cam_init_done;      // OV5640 I2C 配置完成
    wire        cmos_frame_vsync;   // 帧同步 (cam_pclk 域)
    wire        cmos_frame_href;    // 行同步
    wire        cmos_frame_valid;   // 像素有效
    wire [15:0] cmos_frame_data;    // RGB565 像素数据

    ov5640_dri u_ov5640_dri (
        .clk               (pll_50m_clk),     // 50MHz I2C 驱动时钟 (CLK_FREQ=50MHz)
        .rst_n             (rst_n),
        .cam_pclk          (cam_pclk),
        .cam_vsync         (cam_vsync),
        .cam_href          (cam_href),
        .cam_data          (cam_data),
        .cam_rst_n         (cam_rst_n),
        .cam_pwdn          (cam_pwdn),
        .cam_scl           (cam_scl),
        .cam_sda           (cam_sda),
        .capture_start     (cam_init_done),   // I2C 配置完成后再开始采集
        .cmos_h_pixel      (13'd1280),
        .cmos_v_pixel      (13'd720),
        .total_h_pixel     (13'd2570),        // HTS (ebaz4205 验证: 1280×2+10)
        .total_v_pixel     (13'd980),         // VTS (ebaz4205 验证: 720+260)
        .cmos_frame_vsync  (cmos_frame_vsync),
        .cmos_frame_href   (cmos_frame_href),
        .cmos_frame_valid  (cmos_frame_valid),
        .cmos_frame_data   (cmos_frame_data),
        .cam_init_done     (cam_init_done)
    );

    // RGB565 直传 (不经 888 转换, host 端做 RGB565→BGRA)
    assign cam_de     = cmos_frame_valid;
    assign cam_vs     = cmos_frame_vsync;
    assign cam_rgb565 = cmos_frame_data;

    // ====================================================================
    // 6.1 像素打包: 16-bit RGB565 × 8 像素 → 128-bit AXI-Stream
    // ====================================================================
    video_axis_packer u_video_packer (
        .clk           (cam_pclk),
        .rst_n         (rst_n),
        .video_de      (cam_de),
        .video_vs      (cam_vs),
        .pixel_data    (cam_rgb565),
        .m_axis_tdata  (video_packed_tdata),
        .m_axis_tkeep  (video_packed_tkeep),
        .m_axis_tvalid (video_packed_tvalid),
        .m_axis_tready (video_packed_tready),
        .m_axis_tlast  (video_packed_tlast)
    );

    // ====================================================================
    // 6.2 Gate FSM (★ v2.0: vsync 清 FIFO + vsync↓ 开门, 永久开)
    // ====================================================================
    // 开门: host_ready_cam=1 且 cam_vs 下降沿 (FIFO 刚被 vsync 高清空)
    // 关门: host_ready=0 (host 退出)
    // 溢出: 不关门不清 FIFO, 只标记错误帧尾 (header_inserter FILL_DUMMY + E1)
    //   - vsync 高期间清 FIFO (帧头消隐, 无有效数据, 安全)
    //   - SGDMA 提前就位等数据, vsync↓ 数据从干净帧头流入

    // vsync 边沿检测
    reg  cam_vs_d1;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n)
            cam_vs_d1 <= 1'b1;
        else
            cam_vs_d1 <= cam_vs;
    end
    wire cam_vs_neg = cam_vs_d1 && !cam_vs;   // 下降沿 (有效数据开始)
    wire cam_vs_ris = cam_vs && !cam_vs_d1;   // 上升沿 (帧尾, 锁存检测结果)

    // ★ v2.0: 行数/列数检测 (cam_pclk 域)
    // href 边沿检测
    reg  cam_href_d1;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n)
            cam_href_d1 <= 1'b0;
        else
            cam_href_d1 <= cam_href;
    end
    wire href_rise = cam_href && !cam_href_d1;   // 行开始
    wire href_fall = !cam_href && cam_href_d1;   // 行结束

    // 行计数: vsync 高期间复位, href 上升沿 +1
    reg [9:0] row_cnt;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n)
            row_cnt <= 10'd0;
        else if (cam_vs)
            row_cnt <= 10'd0;
        else if (href_rise)
            row_cnt <= row_cnt + 1'b1;
    end

    // 列计数: 每行 href 高期间数 cam_de, 行结束比较 1280, 不匹配锁存
    reg [10:0] col_cnt;
    reg        col_mismatch;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt      <= 11'd0;
            col_mismatch <= 1'b0;
        end else if (cam_vs) begin
            col_cnt      <= 11'd0;
            col_mismatch <= 1'b0;
        end else if (href_fall) begin
            if (col_cnt != 11'd1279) col_mismatch <= 1'b1;
            col_cnt <= 11'd0;
        end else if (cam_href && cam_de) begin
            col_cnt <= col_cnt + 1'b1;
        end
    end

    // vsync 上升沿 (帧尾) 锁存本帧检测结果
    reg  row_status_cam, col_status_cam;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            row_status_cam <= 1'b0;
            col_status_cam <= 1'b0;
        end else if (cam_vs_ris) begin
            row_status_cam <= (row_cnt != 10'd720);
            col_status_cam <= col_mismatch;
        end
    end

    // 行/列状态 CDC: cam_pclk → tlp_clk (供 header_inserter 拼帧尾)
    reg  row_status_tlp_d1, row_status_tlp_d2;
    reg  col_status_tlp_d1, col_status_tlp_d2;
    always @(posedge tlp_clk or negedge rst_n) begin
        if (!rst_n) begin
            row_status_tlp_d1 <= 1'b0;
            row_status_tlp_d2 <= 1'b0;
            col_status_tlp_d1 <= 1'b0;
            col_status_tlp_d2 <= 1'b0;
        end else begin
            row_status_tlp_d1 <= row_status_cam;
            row_status_tlp_d2 <= row_status_tlp_d1;
            col_status_tlp_d1 <= col_status_cam;
            col_status_tlp_d2 <= col_status_tlp_d1;
        end
    end
    wire row_status_tlp = row_status_tlp_d2;
    wire col_status_tlp = col_status_tlp_d2;

    // ★ 写侧溢出锁存: gate 开且 FIFO 接近满 → 置位; vsync↓ 清除
    //   溢出只标记错误帧尾, 不清 FIFO 不关 gate (vsync 每帧清 FIFO 自愈)
    wire fifo_near_full = (fifo_wr_depth >= (VIDEO_FIFO_DEPTH - 64));
    reg  overflow_cam;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n)
            overflow_cam <= 1'b0;
        else if (cam_vs_neg)
            overflow_cam <= 1'b0;
        else if (gate_open && fifo_near_full)
            overflow_cam <= 1'b1;
    end

    // ★ v2.0: gate FSM 简化 — host_ready=0 直接关, vsync↓ 开门永久开
    reg  gate_open;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n)
            gate_open <= 1'b0;
        else if (!host_ready_cam)                       // host 退出 → 关门
            gate_open <= 1'b0;
        else if (!gate_open && cam_vs_neg)              // vsync↓ 开门 (FIFO 刚被清)
            gate_open <= 1'b1;
    end

    // ★ overflow CDC: cam_pclk → tlp_clk (供 header_inserter 标记错误帧尾)
    reg  overflow_tlp_d1, overflow_tlp_d2;
    always @(posedge tlp_clk or negedge rst_n) begin
        if (!rst_n) begin
            overflow_tlp_d1 <= 1'b0;
            overflow_tlp_d2 <= 1'b0;
        end else begin
            overflow_tlp_d1 <= overflow_cam;
            overflow_tlp_d2 <= overflow_tlp_d1;
        end
    end
    wire overflow_tlp = overflow_tlp_d2;

    // ====================================================================
    // 6.3 CDC: cam_pclk(48MHz) → tlp_clk(100MHz)
    //     含 gate 门控: gate 关闭时屏蔽 tvalid + 丢弃数据
    // ====================================================================
    axis_async_fifo #(
        .DEPTH      (VIDEO_FIFO_DEPTH),
        .DATA_WIDTH (AXI_DATA_WIDTH),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH (AXI_STRB_WIDTH),
        .LAST_ENABLE(1),
        .ID_ENABLE  (0),
        .DEST_ENABLE(0),
        .USER_ENABLE(0)
    ) u_axis_video_cdc (
        .s_clk         (cam_pclk),
        .s_rst         (~rst_n | stream_reset_cam | cam_vs),
        .s_axis_tdata  (video_packed_tdata),
        .s_axis_tkeep  (video_packed_tkeep),
        .s_axis_tvalid (video_packed_tvalid && gate_open),   // ★ gate 门控
        .s_axis_tready (video_packed_tready),
        .s_axis_tlast  (video_packed_tlast),
        .m_clk         (tlp_clk),
        .m_rst         (~rst_n | stream_reset | video_vs_tlp),
        .m_axis_tdata  (video_tlp_tdata),
        .m_axis_tkeep  (video_tlp_tkeep),
        .m_axis_tvalid (video_tlp_tvalid),
        .m_axis_tready (video_tlp_tready),
        .m_axis_tlast  (video_tlp_tlast),
        // ★ v2.0: 写侧水位 → 溢出检测
        .s_status_depth(fifo_wr_depth)
    );

    // ====================================================================
    // 6.3 帧头 + 帧尾插入 (★ v2.0: 加帧尾 + 溢出全F填充)
    // ====================================================================
    frame_header_inserter #(
        .DATA_WIDTH    (AXI_DATA_WIDTH),
        .KEEP_WIDTH    (AXI_STRB_WIDTH),
        .FRAME_BYTES   (1843200),
        .HEADER_BYTES  (16),
        .TAIL_BYTES    (16),
        .FRAME_ID_W    (16),
        .EXPECTED_ROWS (720),
        .EXPECTED_COLS (1280)
    ) u_frame_header_inserter (
        .clk           (tlp_clk),
        .rst           (~rst_n | stream_reset),
        .s_axis_tdata  (video_tlp_tdata),
        .s_axis_tkeep  (video_tlp_tkeep),
        .s_axis_tvalid (video_tlp_tvalid),
        .s_axis_tready (video_tlp_tready),
        .s_axis_tlast  (video_tlp_tlast),
        .m_axis_tdata  (video_hdr_tdata),
        .m_axis_tkeep  (video_hdr_tkeep),
        .m_axis_tvalid (video_hdr_tvalid),
        .m_axis_tready (video_hdr_tready),
        .m_axis_tlast  (video_hdr_tlast),
        // ★ v2.0
        .overflow_in   (overflow_tlp),
        .frame_id      (frame_id),
        .row_status    (row_status_tlp),
        .col_status    (col_status_tlp),
        .frame_active  (frame_active)
    );

    // ====================================================================
    // 6.4 宽度转换: 128-bit → 256-bit (对齐 SGDMA C2H)
    // ====================================================================
    axis_width_up u_axis_c2h_up (
        .clk           (tlp_clk),
        .rst           (~rst_n | stream_reset | video_vs_tlp),
        .s_axis_tdata  (video_hdr_tdata),
        .s_axis_tkeep  (video_hdr_tkeep),
        .s_axis_tvalid (video_hdr_tvalid),
        .s_axis_tready (video_hdr_tready),
        .s_axis_tlast  (video_hdr_tlast),
        .m_axis_tdata  (video_256_tdata),
        .m_axis_tkeep  (video_256_tkeep),
        .m_axis_tvalid (video_256_tvalid),
        .m_axis_tready (video_256_tready),
        .m_axis_tlast  (video_256_tlast)
    );

    // ====================================================================
    // 6.5 直连 SGDMA C2H
    // ====================================================================
    assign axis_c2h_tdata  = video_256_tdata;
    assign axis_c2h_tkeep  = video_256_tkeep;
    assign axis_c2h_tvalid = video_256_tvalid;
    assign axis_c2h_tlast  = video_256_tlast;
    assign video_256_tready = axis_c2h_tready;

    // ====================================================================
    // 6.6 frame_id: 来自 logic_dma (vsync 下降沿计数, BAR2 Status[15:0])
    //     注: 不再使用 header_inserter tlast 计数, 改用 vsync 驱动
    // ====================================================================


    // ========================================================================
    // 第 6B 节: H2C 数据路径 (★ 保留, 供未来控制通道)
    // ========================================================================
    // 数据流:
    //   SGDMA H2C(256b) → axis_width_down(256→128)
    //     → async_fifo(tlp_clk, 128深, 缓冲)
    //       → [terminated: m_axis_tready = 0]
    //
    // 当前状态: FIFO 输出 terminated, H2C 传输会因 FIFO 满而反压 SGDMA。
    // 未来使用: 将 h2c_fifo_tready 连接到控制模块即可启用 H2C 通道。
    //
    // H2C 描述符由 logic_dma BAR2 寄存器管理 (0x10 H2C_Addr, 0x14 H2C_Len)

    // H2C FIFO 输出 (当前 terminated)
    wire                      h2c_fifo_tvalid;
    wire [AXI_DATA_WIDTH-1:0] h2c_fifo_tdata;
    wire                      h2c_fifo_tlast;
    wire [AXI_STRB_WIDTH-1:0] h2c_fifo_tkeep;
    wire                      h2c_fifo_tready = 1'b0;  // ★ 当前无消费者

    // H2C overhead 锁存 (供 BAR2 0x18/0x1C 读取)
    always @(posedge tlp_clk or negedge rst_n) begin
        if (!rst_n)
            h2c_overhead_reg <= 64'd0;
        else if (axis_h2c_tvalid)
            h2c_overhead_reg <= h2c_overhead;
    end

    // H2C: 256 → 128 位宽转换 (高128先出, 低128后出)
    axis_width_down u_axis_h2c_down (
        .clk           (tlp_clk),
        .rst           (~rst_n),
        .s_axis_tdata  (axis_h2c_tdata),
        .s_axis_tkeep  (axis_h2c_tkeep),
        .s_axis_tvalid (axis_h2c_tvalid),
        .s_axis_tready (axis_h2c_tready),
        .s_axis_tlast  (axis_h2c_tlast),
        .m_axis_tdata  (h2c_128_tdata),
        .m_axis_tkeep  (h2c_128_tkeep),
        .m_axis_tvalid (h2c_128_tvalid),
        .m_axis_tready (h2c_128_tready),
        .m_axis_tlast  (h2c_128_tlast)
    );

    // H2C 缓冲 FIFO (tlp_clk 域, 同频同步 FIFO)
    // 未来如需跨时钟域, 将 m_clk 改为目标时钟即可
    axis_async_fifo #(
        .DEPTH      (128),
        .DATA_WIDTH (AXI_DATA_WIDTH),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH (AXI_STRB_WIDTH),
        .LAST_ENABLE(1),
        .ID_ENABLE  (0),
        .DEST_ENABLE(0),
        .USER_ENABLE(0)
    ) u_axis_h2c_fifo (
        .s_clk         (tlp_clk),
        .s_rst         (~rst_n),
        .s_axis_tdata  (h2c_128_tdata),
        .s_axis_tkeep  (h2c_128_tkeep),
        .s_axis_tvalid (h2c_128_tvalid),
        .s_axis_tready (h2c_128_tready),
        .s_axis_tlast  (h2c_128_tlast),
        .m_clk         (tlp_clk),
        .m_rst         (~rst_n),
        .m_axis_tdata  (h2c_fifo_tdata),
        .m_axis_tkeep  (h2c_fifo_tkeep),
        .m_axis_tvalid (h2c_fifo_tvalid),
        .m_axis_tready (h2c_fifo_tready),    // ★ = 1'b0, terminated
        .m_axis_tlast  (h2c_fifo_tlast)
    );

    // H2C 描述符 ready: 当 FIFO 有空间时 (watermark-based)
    // 简化: 始终 ready, FIFO 满时会自动反压 width_down → SGDMA
    assign axis_h2c_desc_ready = 1'b1;


    // ========================================================================
    // 第 7 节: HDMI 彩条显示 (独立时钟域, 不受 C2H 反压影响)
    // ========================================================================
    // 数据流: video_display → video_driver → DVI_TX → TMDS
    // 分辨率: 1280×720 @ 60Hz

    // ----- HDMI PLL: 25MHz → 75MHz + 375MHz -----
    wire        pixel_clk;          // 75MHz 像素时钟
    wire        pixel_clk_5x;       // 375MHz TMDS 串行时钟
    wire        HDMI_lock;          // HDMI PLL 锁定

    Gowin_PLL_HDMI uHDMIpll (
        .lock    (HDMI_lock),
        .clkout0 (pixel_clk),
        .clkout1 (pixel_clk_5x),
        .clkin   (sys_clkin),
        .init_clk(sys_clkin),
        .reset   (~hard_rst_n)
    );

    // 视频时序驱动: 生成 HS/VS/DE + 像素坐标
    video_driver u_video_driver (
        .pixel_clk  (pixel_clk),
        .sys_rst_n  (rst_n & HDMI_lock),
        .video_hs   (video_hs),
        .video_vs   (video_vs),
        .video_de   (video_de),
        .video_rgb  (video_rgb),
        .data_req   (),
        .pixel_xpos (pixel_xpos_w),
        .pixel_ypos (pixel_ypos_w),
        .pixel_data (pixel_data_w)
    );

    // 彩条生成: 根据坐标生成 5 色竖条
    video_display u_video_display (
        .pixel_clk  (pixel_clk),
        .sys_rst_n  (rst_n & HDMI_lock),
        .pixel_xpos (pixel_xpos_w),
        .pixel_ypos (pixel_ypos_w),
        .pixel_data (pixel_data_w)
    );

    // HDMI / DVI 发送
    DVI_TX_Top u_HDMI (
        .I_rst_n        (rst_n & HDMI_lock),
        .I_serial_clk   (pixel_clk_5x),
        .I_rgb_clk      (pixel_clk),
        .I_rgb_vs       (video_vs),
        .I_rgb_hs       (video_hs),
        .I_rgb_de       (video_de),
        .I_rgb_r        (video_rgb[23:16]),
        .I_rgb_g        (video_rgb[15: 8]),
        .I_rgb_b        (video_rgb[ 7: 0]),
        .O_tmds_clk_p   (tmds_clk_p),
        .O_tmds_clk_n   (tmds_clk_n),
        .O_tmds_data_p  (tmds_data_p),
        .O_tmds_data_n  (tmds_data_n)
    );


    // ========================================================================
    // 第 8 节: 心跳 & LED 指示
    // ========================================================================
    //   [0] = heart0: 50MHz 时钟, 半秒翻转
    //   [1] = heart1: 100MHz 时钟, 1秒翻转
    //   [2] = heart2: 50MHz 时钟, 1秒翻转 (原 ui_clk, DDR3 已删除)
    //   [3] = ~pcie_rst_n: PCIe 复位状态
    //   [4] = pcie_linkup: PCIe 链路建立
    //   [5] = h2c_run: H2C 进行中 (始终 0, H2C 已禁用)
    //   [6] = c2h_run: C2H 进行中
    //   [7] = pcie_linkup: 链路已建立 (原 ddr_init, DDR3 已删除)

    wire heart0, heart1, heart2;

    // heart0: 50MHz → 半秒翻转, 计数 = 25,000,000
    heartbeat #(.COUNT_MAX(25_000_000 - 1)) u_heart0 (
        .clk   (pll_50m_clk),
        .rst_n (rst_n),
        .heart (heart0)
    );

    // heart1: 100MHz → 1秒翻转, 计数 = 100,000,000
    heartbeat #(.COUNT_MAX(100_000_000 - 1)) u_heart1 (
        .clk   (tlp_clk),
        .rst_n (rst_n),
        .heart (heart1)
    );

    // heart2: 50MHz → 1秒翻转 (原 ui_clk 约 100MHz)
    heartbeat #(.COUNT_MAX(50_000_000 - 1)) u_heart2 (
        .clk   (pll_50m_clk),
        .rst_n (rst_n),
        .heart (heart2)
    );

    assign LED[0] = heart0;
    assign LED[1] = heart1;
    assign LED[2] = heart2;
    assign LED[3] = ~pcie_rst_n;      // 亮=复位中
    assign LED[4] = pcie_linkup;      // 亮=PCIe 已连接
    assign LED[5] = h2c_run;          // H2C 运行 (始终 0)
    assign LED[6] = c2h_run;          // C2H 运行
    assign LED[7] = pcie_linkup;      // (原 ddr_init)

endmodule

`default_nettype wire
