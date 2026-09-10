// ============================================================================
// sgdma_core.h — sgdmaVideoShakeHand v1.4 SGDMA 核心库 (环形 5 帧持续流)
// ============================================================================
// 功能:
//   - PCIe SGDMA 设备初始化 / BAR0/BAR2 映射 / DMA buffer 分配
//   - ★ BAR2 握手协议: vsync↓ → frame_id↑ → Host 设 desc 启 SGDMA → host_ready
//   - ★ 环形 5 帧缓冲: desc[0..4]→buf[0..4] (各含 FLAG_LAST, 每帧重配+重启)
//   - ★ 显示线程滞后 RING_DISPLAY_LAG=3 帧, 握手线程 10μs busy-wait
//   - Benchmark: SGDMA 读测速 + BAR2 R/W 延迟测量
//   - 日志系统 + 信号处理
//
// 依赖:
//   utils/log.h — 日志系统
// ============================================================================

#ifndef SGDMA_CORE_H
#define SGDMA_CORE_H

#include <stdint.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- 视频参数 ----
#define VIDEO_WIDTH      1280
#define VIDEO_HEIGHT     720
#define PIXEL_BYTES      2
#define FRAME_SIZE       (VIDEO_WIDTH * VIDEO_HEIGHT * PIXEL_BYTES)  // 1,843,200

// ---- 帧字节数 (含 16B 帧头 + 16B 帧尾) ★ v2.0 ----
#define FRAME_HEADER_BYTES  16                                      // 16B magic header
#define FRAME_TAIL_BYTES    16                                      // ★ v2.0: 16B 帧尾 (状态标记)
#define FRAME_TOTAL_BYTES   (FRAME_SIZE + FRAME_HEADER_BYTES + FRAME_TAIL_BYTES)  // 1,843,232
#define FRAME_BUF_BYTES     ((FRAME_TOTAL_BYTES + 4095UL) & ~4095UL) // 1,847,296 分配/映射 (4KB 对齐)

// ---- DMA 描述符配置 (1 desc = 1 帧) ----
#define RING_DESCS        8            // 8 个描述符 (环形)
#define DESC_SIZE         32           // 每描述符 32 字节
#define DESC_TOTAL_BYTES  (8192)       // 描述符表总大小 (含 poll 区)

// ---- DMA 缓冲: 8 帧环形 ----
// ★ v1.5: 5→8 帧, 滞后 3 帧保护窗口从 1 帧(16.4ms) 增到 4 帧(65.6ms),
//   消除 BMP 保存/慢渲染期间 buffer 被覆盖导致的撕裂
#define RING_FRAMES       8
#define RING_DISPLAY_LAG  3            // 显示线程滞后 3 帧
#define BUF_TOTAL_SIZE    (RING_FRAMES * FRAME_BUF_BYTES)  // ≈ 28.2 MB

// ---- BAR2 握手寄存器 ----
#define BAR2_CTRL_HOST_READY  (1 << 4)   // Ctrl bit4
#define BAR2_STATUS_VSYNC     (1 << 0)   // Status bit0
#define BAR2_STATUS_FRAME_ID  0x0000FFFF // Status [15:0]

// ---- ★ v2.0: 版本号 (FPGA BAR2 0x0C 只读, 启动时验证) ----
//   v2.0: 新帧头帧尾格式 (AAABACAD / FAFBFCFD + 行列状态) + gate 简化
#define HOST_VERSION         0x00000200  // major=2, minor=0
#define HOST_VERSION_MAJOR   ((HOST_VERSION >> 8) & 0xFF)
#define HOST_VERSION_MINOR   (HOST_VERSION & 0xFF)

// ---- 默认参数 ----
#define DEFAULT_DEVNODE     "/dev/gowin_pcie_demo"
#define TIMEOUT_C2H_MS      2000
#define HANDSHAKE_POLL_US   100          // 握手轮询间隔 100μs (usleep 温和轮询, 省 CPU)

// ---- PCIe ----
#define PCIE_READY       (0xaa009719)
#define CREDIT_MAX       (0x3FF)
#define SGDMA_STOP       0x0000
#define SGDMA_START_POLL ((1 << 0) | (1 << 1))

// ---- BAR ----
#define BAR0_SIZE        (1024 * 16)
#define BAR2_SIZE        (1024 * 2)

// ---- 描述符标志 ----
#define SET_FLAG_STOP  (1 << 0)
#define SET_FLAG_EOP   (1 << 1)
#define SET_FLAG_COMP  (1 << 2)
#define FLAG_LAST      (SET_FLAG_STOP | SET_FLAG_EOP | SET_FLAG_COMP)

// ---- BAR0 结构体 (不变) ----
typedef struct __attribute__((packed, aligned(32))) {
    volatile uint32_t id, ctrl, ctrl_w1s, ctrl_w1c;
    volatile uint32_t addr_desc_lo, addr_desc_hi, addr_poll_lo, addr_poll_hi;
    volatile uint32_t desc_count, rsv_24, num_desc_adj, rsv_2c;
    volatile uint32_t status0, status1, rsv_38[5], credit, rsv_50[44];
} GowinDMAChannel;

typedef struct __attribute__((packed, aligned(32))) {
    volatile uint32_t id, ctrl_init, stat_init, rsv_0c[61];
} GowinControl;

typedef struct __attribute__((packed, aligned(32))) {
    GowinDMAChannel   h2c[16];
    GowinDMAChannel   c2h[16];
    volatile uint32_t rsv_ctrl_pre[64];
    GowinControl      ctrl;
    volatile uint32_t rsv_ctrl_post[896];
    volatile uint32_t rsv[1024];
} GowinBar0;

// ---- BAR2 结构体 — v1.0 Handshake ----
typedef struct __attribute__((packed, aligned(32))) {
    volatile uint32_t ctrl;              // 0x00 — bit4=host_ready
    volatile uint32_t status;            // 0x04 — bit0=vsync, [15:0]=frame_id
    volatile uint32_t stream_reset;      // 0x08 — bit0=stream_reset
    volatile uint32_t version;           // 0x0C — ★ v1.0: FPGA 版本号 (只读)
    volatile uint32_t addr_ddr_h2c;      // 0x10
    volatile uint32_t leng_ddr_h2c;      // 0x14
    volatile uint32_t rsv_18[2];         // 0x18-0x1F
    volatile uint32_t rsv_20[492];       // 0x20-0x7FF
} GowinBar2;

// ---- 描述符 (从 SGDMADDR5v0/host 拷贝) ----
typedef struct __attribute__((packed, aligned(32))) {
    volatile uint32_t flags, length;
    volatile uint32_t addr_src_lo, addr_src_hi;
    volatile uint32_t addr_dst_lo, addr_dst_hi;
    volatile uint32_t next_lo, next_hi;
} GowinDescriptor;

// ---- SGDMA 设备上下文 (v1.0 Handshake) ----
typedef struct {
    int fd;
    volatile GowinBar0 *bar0;
    volatile GowinBar2 *bar2;

    // DMA 帧缓冲: 5 帧环形
    uint8_t  *buf_mem[RING_FRAMES];   // 虚拟地址
    uint64_t  buf_phys[RING_FRAMES];  // 总线地址
    int       buf_idx;                // SGDMA 正在写入的 slot (0..4)

    // 描述符: desc[0..4]→buf[0..4] (各含 FLAG_LAST)
    volatile GowinDescriptor *desc_mem;
    uint64_t  desc_phys;
    uint32_t  desc_size;

    // 握手状态
    uint16_t  last_frame_id;          // 上次看到的 frame_id

    // ★ v1.4 环形 5 帧共享状态 (握手线程写, 渲染线程读)
    volatile int       hs_in_flight;            // 1=有一帧 SGDMA 传输进行中
    volatile int       hs_in_flight_slot;       // 进行中传输的 slot (0..4)
    volatile int       ring_write_count;        // 累计完成帧数 (单调增)
    volatile int       ring_ready[RING_FRAMES];     // 1=slot 有完整帧可渲染
    volatile uint16_t  ring_frame_id[RING_FRAMES];  // slot 对应 frame_id

    // Benchmark 结果
    double    bar2_read_us;          // BAR2 单次读延迟 (μs)
    double    bar2_write_us;         // BAR2 单次写延迟 (μs)
} SGDMAContext;

// ---- 全局信号标志 ----
extern volatile sig_atomic_t g_exit;
extern int g_save_bmp;
extern int g_frame_dump;   // ★ v2.0: -vv 每帧 dump 帧头帧尾分析

// ============================================================================
// API 函数 (v1.0 Handshake)
// ============================================================================

// 初始化 SGDMA 设备 + DMA 缓冲 + 描述符
int sgdma_init(SGDMAContext *ctx, const char *devnode);

// ★ v1.4 环形 5 帧 API:
//   - sgdma_handshake_tick(): 非阻塞, 供握手线程 10μs busy-wait 循环调用
//     1) 完成检测: poll 字回写 → 清 desc flags → ring_ready[slot]=1 → 清 in_flight
//     2) 启动新帧: frame_id↑ 且 !in_flight → setup desc → start SGDMA
//     返回: 1=本 tick 有动作, 0=无动作
//   - sgdma_get_display_frame(): 渲染线程调用, 返回滞后 RING_DISPLAY_LAG 帧的 buffer
//     返回 NULL 表示尚未有可显示的帧
int sgdma_handshake_tick(SGDMAContext *ctx);
uint8_t *sgdma_get_display_frame(SGDMAContext *ctx, uint16_t *out_frame_id);

// 获取 BAR2 Status
static inline uint32_t sgdma_bar2_status(SGDMAContext *ctx) {
    return ctx->bar2->status;
}
static inline uint16_t sgdma_frame_id(SGDMAContext *ctx) {
    return (uint16_t)(ctx->bar2->status & 0xFFFF);
}
static inline int sgdma_vsync_state(SGDMAContext *ctx) {
    return (ctx->bar2->status & 1) ? 1 : 0;
}

// ★ Benchmark: BAR2 读写延迟测量 (100 次取平均)
int sgdma_benchmark_bar2_latency(SGDMAContext *ctx);

// 保存帧为 BMP
int sgdma_save_frame_bmp(const uint8_t *pixels, uint16_t frame_id,
                          int width, int height);

// 释放所有资源
void sgdma_cleanup(SGDMAContext *ctx);

#ifdef __cplusplus
}
#endif

#endif // SGDMA_CORE_H
