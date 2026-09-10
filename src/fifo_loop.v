module fifo_loop(
    input               clk,
    input               rstn,
    // H2C AXI-Stream (from SGDMA)
    input               h2c_run,
    output reg          m_axis_h2c_tready = 0,
    input               m_axis_h2c_tvalid,
    input      [255:0]  m_axis_h2c_tdata,
    input               m_axis_h2c_tlast,
    input      [ 31:0]  m_axis_h2c_tuser,
    input      [ 31:0]  m_axis_h2c_tkeep,
    input      [ 63:0]  h2c_overhead,
    // C2H AXI-Stream (to SGDMA)
    input               c2h_run,
    input               s_axis_c2h_tready,
    output              s_axis_c2h_tvalid,
    output reg          s_axis_c2h_tlast  = 0,
    output     [255:0]  s_axis_c2h_tdata,
    output     [ 31:0]  s_axis_c2h_tuser,
    output     [ 31:0]  s_axis_c2h_tkeep,
    output reg          c2h_overhead_valid = 0,
    output reg [ 63:0]  c2h_overhead_data  = 64'h01_02_03_04_aa_bb_cc_dd
);

// ============================================================
// FIFO IP (256-bit × 512-deep, FWFT, synchronous)
// ============================================================
wire            fifo_full;
wire            fifo_empty;
wire [255:0]    fifo_rdata;
reg             fifo_wen;
reg  [255:0]    fifo_wdata;
wire            fifo_ren;

fifo_top u_fifo_top(
    .Data   (fifo_wdata),
    .WrClk  (clk),
    .RdClk  (clk),
    .WrEn   (fifo_wen),
    .RdEn   (fifo_ren),
    .Q      (fifo_rdata),
    .Empty  (fifo_empty),
    .Full   (fifo_full)
);

// ============================================================
// Write side: H2C → FIFO
// ============================================================
reg [15:0] wr_beats;     // total beats written
reg [15:0] wr_packets;   // total packets written (tlast count)

// tready: gate by h2c_run + FIFO not full
always@(posedge clk or negedge rstn) begin
    if (!rstn) begin
        m_axis_h2c_tready <= 0;
    end else begin
        m_axis_h2c_tready <= h2c_run && !fifo_full;
    end
end

// FIFO write
always@(posedge clk or negedge rstn) begin
    if (!rstn) begin
        fifo_wen   <= 0;
        fifo_wdata <= 0;
    end else begin
        fifo_wen   <= m_axis_h2c_tvalid && m_axis_h2c_tready;
        fifo_wdata <= m_axis_h2c_tdata;
    end
end

// Write counters: reset when H2C channel idle
always@(posedge clk or negedge rstn) begin
    if (!rstn) begin
        wr_beats   <= 0;
        wr_packets <= 0;
    end else if (!h2c_run) begin
        wr_beats   <= 0;
        wr_packets <= 0;
    end else if (m_axis_h2c_tvalid && m_axis_h2c_tready) begin
        wr_beats <= wr_beats + 1;
        if (m_axis_h2c_tlast)
            wr_packets <= wr_packets + 1;
    end
end

// ============================================================
// Read side: FIFO → C2H  (standard FIFO, 1-cycle read latency)
//
// 标准 FIFO 时序：RdEn 后 1 拍 Q 才输出数据
//   需要一次"预取"（prefetch）把首个数据加载到输出寄存器 Q，
//   之后流水线：当前拍 tvalid&&tready 握手 → fifo_ren 预取下一拍。
//   tvalid 用组合逻辑（assign），避免寄存器延迟导致丢拍。
// ============================================================
reg [15:0] rd_beats;     // total beats read
reg [15:0] rd_packets;
reg        rd_active;    // Q 输出寄存器已装载有效数据

assign s_axis_c2h_tdata = fifo_rdata;
assign s_axis_c2h_tuser = 0;
assign s_axis_c2h_tkeep = 32'hffff_ffff;

// c2h_done: 所有已写入的 beat 都已读出（包完成后才有效）
wire c2h_done;
assign c2h_done = (wr_packets > 0) && (rd_beats >= wr_beats);

// tvalid (组合逻辑): Q 上有有效数据 = Q 已被装载 且 C2H 运行中 且未完成
assign s_axis_c2h_tvalid = c2h_run && rd_active && !c2h_done;

// FIFO read enable:
//   PRIME : rd_active=0, FIFO 有数据 → 预取第一拍到 Q（不握手）
//   NORMAL: rd_active=1, SGDMA 准备好接收 → 本次握手 + 预取下一拍
assign fifo_ren = (!rd_active && c2h_run && !fifo_empty && !c2h_done)
                || (rd_active && s_axis_c2h_tready);

// rd_active: 跟踪 Q 输出寄存器是否已被装载
//   置 1：fifo_ren 有效 → 下一拍 Q 有数据
//   清 0：C2H 空闲 或 传输完成
always@(posedge clk or negedge rstn) begin
    if (!rstn) begin
        rd_active <= 0;
    end else if (!c2h_run) begin
        rd_active <= 0;
    end else if (c2h_done) begin
        // 所有数据已读完 → 关闭
        rd_active <= 0;
    end else if (fifo_ren) begin
        // 预取或正常读 → 下一拍 Q 有数据
        rd_active <= 1;
    end
end

// tlast: 当前拍是最后一拍
// (所有数据已写入，且这是最后一拍待读数据)
always@(posedge clk or negedge rstn) begin
    if (!rstn) begin
        s_axis_c2h_tlast <= 0;
    end else if (s_axis_c2h_tvalid) begin
        s_axis_c2h_tlast <= (wr_packets > 0) && (wr_beats - rd_beats == 1);
    end else begin
        s_axis_c2h_tlast <= 0;
    end
end

// Read counters: increment on handshake, reset when C2H idle
always@(posedge clk or negedge rstn) begin
    if (!rstn) begin
        rd_beats   <= 0;
        rd_packets <= 0;
    end else if (!c2h_run) begin
        rd_beats   <= 0;
        rd_packets <= 0;
    end else if (s_axis_c2h_tvalid && s_axis_c2h_tready) begin
        rd_beats <= rd_beats + 1;
        if (s_axis_c2h_tlast)
            rd_packets <= rd_packets + 1;
    end
end

// C2H overhead
always@(posedge clk or negedge rstn) begin
    if (!rstn) begin
        c2h_overhead_valid <= 0;
        c2h_overhead_data  <= 64'h01_02_03_04_aa_bb_cc_dd;
    end else begin
        c2h_overhead_valid <= c2h_run && (rd_beats == 0)
                              && s_axis_c2h_tvalid && s_axis_c2h_tready;
        if (c2h_overhead_valid)
            c2h_overhead_data <= c2h_overhead_data + 1;
    end
end

endmodule
