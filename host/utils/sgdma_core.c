// ============================================================================
// sgdma_core.c — sgdmaVideoShakeHand v1.4 SGDMA 核心库 (环形 5 帧持续流)
// ============================================================================
// v1.4 环形 5 帧改动 vs v1.3:
//   - ★ host_ready 纯锁存: init 时写 1 永久开门, 不再每帧重写
//   - ★ 环形 5 帧缓冲: desc[0..4]→buf[0..4], 各含 FLAG_LAST 自动停
//   - ★ 握手线程 10μs busy-wait: 每帧重配 (清 flags + setup + start)
//   - ★ desc.length = FRAME_TOTAL_BYTES (3,686,432, 含 16B 帧头 + 16B 帧尾)
//   - ★ v2.0: get_display_frame 检测帧尾标记, 溢出帧丢弃取相邻帧
//   - ★ 显示线程滞后 RING_DISPLAY_LAG=3 帧
//   - ★ 保留: BAR2 握手, Benchmark, BMP dump
// ============================================================================

#include "sgdma_core.h"
#include "log.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>

// ---- 驱动 uapi (与 driver/gowin_demo_drv.c 一致) ----
#define GOWIN_BAR_READ_DWORD     _IOWR('G', 0x1, unsigned long)
#define GOWIN_BAR_WRITE_DWORD    _IOWR('G', 0x2, unsigned long)
#define GOWIN_CONFIG_READ_DWORD  _IOWR('G', 0x3, unsigned long)
#define GOWIN_SWITCH_BAR_OR_MEM  _IOW ('G', 0x5, unsigned long)
#define GOWIN_REQUEST_DMA_MEM    _IOWR('G', 0x6, unsigned long)
#define GOWIN_RELEASE_DMA_MEM    _IOW ('G', 0x7, unsigned long)

struct gowin_ioctl_param {
    union {
        uint32_t argv[8];
        struct {
            int32_t  bar_idx; uint32_t bar_type; uint32_t bar_offset;
            union { uint32_t bar_dword; uint16_t bar_word; uint8_t bar_byte; };
        };
        struct {
            uint32_t cfg_type; uint32_t cfg_where;
            union { uint32_t cfg_dword; uint16_t cfg_word; uint8_t cfg_byte; };
        };
        struct {
            int32_t dma_idx; uint32_t dma_size; uint32_t dma_realloc;
            uint32_t nu_2; void *dma_addr; uint64_t dma_handle;
        };
        struct { int32_t index; uint32_t dma_select; };
    };
};

// ---- 全局 ----
volatile sig_atomic_t g_exit = 0;
int g_save_bmp = 0;      // -vvb: dump each frame as BMP
int g_frame_dump = 0;    // ★ v2.0: -vv 每帧 dump 帧头帧尾分析

// ============================================================================
// 硬件访问辅助
// ============================================================================

static void bar_writel(int fd, int bar, uint32_t offset, uint32_t value) {
    struct gowin_ioctl_param p = {0};
    p.bar_idx = (bar < 0 || bar > 5) ? 0 : bar;
    p.bar_type = 2; p.bar_offset = offset; p.bar_dword = value;
    if (ioctl(fd, GOWIN_BAR_WRITE_DWORD, &p))
        SLOG_ERROR("bar_writel(bar=%d,off=0x%04x,val=0x%08x): %s",
                   bar, offset, value, strerror(errno));
    else
        SLOG_TRACE("bar_writel(bar=%d,off=0x%04x,val=0x%08x)", bar, offset, value);
}

static uint64_t request_mem(int fd, int index, uint32_t size) {
    struct gowin_ioctl_param p = {0};
    p.dma_idx = index; p.dma_size = size; p.dma_realloc = 1;
    if (ioctl(fd, GOWIN_REQUEST_DMA_MEM, &p))
        SLOG_ERROR("request_mem(idx=%d,size=%u): %s", index, size, strerror(errno));
    return p.dma_handle;
}

static void release_mem(int fd, int index) {
    struct gowin_ioctl_param p = {0};
    p.dma_idx = index;
    if (ioctl(fd, GOWIN_RELEASE_DMA_MEM, &p))
        SLOG_WARN("release_mem(idx=%d): %s", index, strerror(errno));
}

static void switch_bar_or_mem(int fd, int bar, int index) {
    struct gowin_ioctl_param p = {0};
    p.dma_select = (bar == 0 ? 1 : 0); p.index = index;
    if (ioctl(fd, GOWIN_SWITCH_BAR_OR_MEM, &p))
        SLOG_WARN("switch_bar_or_mem(%s,%d): %s",
                  bar ? "BAR" : "MEM", index, strerror(errno));
}

static void *mmap_mem(int fd, int index, size_t length) {
    switch_bar_or_mem(fd, 0, index);
    void *ptr = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        SLOG_ERROR("mmap_mem(idx=%d,len=%zu): %s", index, length, strerror(errno));
        return NULL;
    }
    SLOG_DEBUG("mmap_mem(idx=%d,len=%zu) -> %p", index, length, ptr);
    return ptr;
}

static void *mmap_bar(int fd, int index, size_t length) {
    switch_bar_or_mem(fd, 1, index);
    void *ptr = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        SLOG_ERROR("mmap_bar(idx=%d,len=%zu): %s", index, length, strerror(errno));
        return NULL;
    }
    SLOG_DEBUG("mmap_bar(idx=%d,len=%zu) -> %p", index, length, ptr);
    return ptr;
}

// ============================================================================
// ★ 环形描述符设置 (v1.4: 5 desc → 5 buf, 各含 FLAG_LAST)
// ============================================================================
// desc[0..4] → buf[0..4], 均含 FLAG_LAST
// SGDMA 每完成一个 desc 自动停止, Host 每帧重配 + 重启
static void setup_ring_descriptors(volatile GowinDescriptor *desc_buf,
                                   uint64_t desc_phys,
                                   uint64_t buf_phys[RING_FRAMES],
                                   uint32_t frame_bytes) {
    SLOG_DEBUG("setup_ring_descriptors: %d descs, %u bytes/frame (incl. header)",
               RING_FRAMES, frame_bytes);

    uint32_t len_be   = __builtin_bswap32(frame_bytes);
    uint32_t flags_be = __builtin_bswap32(FLAG_LAST);  // SGDMA 按大端读 desc

    for (int i = 0; i < RING_FRAMES; i++) {
        uint64_t next_phys = desc_phys + ((i + 1) % RING_FRAMES) * DESC_SIZE;

        desc_buf[i].flags       = flags_be;
        desc_buf[i].length      = len_be;
        desc_buf[i].addr_src_lo = 0;
        desc_buf[i].addr_src_hi = 0;
        desc_buf[i].addr_dst_lo = __builtin_bswap32((uint32_t)(buf_phys[i] & 0xFFFFFFFF));
        desc_buf[i].addr_dst_hi = __builtin_bswap32((uint32_t)(buf_phys[i] >> 32));
        desc_buf[i].next_lo     = __builtin_bswap32((uint32_t)(next_phys & 0xFFFFFFFF));
        desc_buf[i].next_hi     = __builtin_bswap32((uint32_t)(next_phys >> 32));
    }

    // desc[5..]: 清零 (备用)
    for (int i = RING_FRAMES; i < (int)(DESC_TOTAL_BYTES / DESC_SIZE); i++) {
        desc_buf[i].flags  = 0;
        desc_buf[i].length = 0;
    }
}

// ============================================================================
// ★ SGDMA C2H 通道配置 (v1.4, 单 desc 模式)
// ============================================================================
static void setup_c2h_channel(SGDMAContext *ctx, int desc_idx) {
    uint64_t desc_phys = ctx->desc_phys + desc_idx * DESC_SIZE;

    ctx->bar0->c2h[0].ctrl         = SGDMA_STOP;
    ctx->bar0->c2h[0].addr_desc_lo = (uint32_t)(desc_phys & 0xFFFFFFFF);
    ctx->bar0->c2h[0].addr_desc_hi = (uint32_t)(desc_phys >> 32);
    // poll 地址: desc 表之后
    uint64_t poll_phys = ctx->desc_phys + DESC_TOTAL_BYTES - 64;
    ctx->bar0->c2h[0].addr_poll_lo = (uint32_t)(poll_phys & 0xFFFFFFFF);
    ctx->bar0->c2h[0].addr_poll_hi = (uint32_t)(poll_phys >> 32);
    ctx->bar0->c2h[0].num_desc_adj = 0;     // 单 desc
    ctx->bar0->c2h[0].credit       = 1;     // 1 个 desc = 1 credit

    SLOG_DEBUG("C2H channel: desc[%d] phys=0x%lx poll_phys=0x%lx",
               desc_idx, (unsigned long)desc_phys, (unsigned long)poll_phys);
}

static void start_sgdma_c2h(SGDMAContext *ctx) {
    ctx->bar0->c2h[0].ctrl = SGDMA_START_POLL;
    __sync_synchronize();
    (void)ctx->bar2->status;
}

static int wait_sgdma_done(SGDMAContext *ctx, int timeout_ms) {
    uint32_t poll_addr_off = DESC_TOTAL_BYTES - 64;
    volatile uint32_t *poll = (volatile uint32_t *)((uint8_t *)ctx->desc_mem + poll_addr_off);

    int elapsed = 0;
    while (elapsed < timeout_ms * 1000) {
        if (*poll != 0) {
            *poll = 0;  // 清零供下次使用
            __sync_synchronize();
            return 0;
        }
        usleep(100);
        elapsed += 100;
    }
    return -1;  // 超时
}

// ============================================================================
// ★ v2.0 环形握手 (非阻塞) — 供握手线程调用
// ============================================================================
// 每 tick 做一件事: 完成检测 → poll 回写 → 标记 slot 可渲染 + 立即 start 下一帧
//   SGDMA 提前就位等下一帧数据 (不等 frame_id), 消除 host 启动延迟导致的帧头偏移
// 关键: hs_in_flight 保持 1 (SGDMA 永远在搬或等数据), 搬完立即重配+重启
int sgdma_handshake_tick(SGDMAContext *ctx) {
    if (!ctx || ctx->fd < 0) return 0;

    uint32_t poll_addr_off = DESC_TOTAL_BYTES - 64;
    volatile uint32_t *poll = (volatile uint32_t *)((uint8_t *)ctx->desc_mem + poll_addr_off);

    // ---- 完成检测: poll 回写 → 标记 slot 可渲染 + 立即 start 下一帧 ----
    if (ctx->hs_in_flight) {
        if (*poll != 0) {
            *poll = 0;                            // 清零供下次使用
            __sync_synchronize();

            int slot = ctx->hs_in_flight_slot;
            ctx->ring_ready[slot] = 1;            // 该 slot 有完整帧可渲染
            ctx->ring_write_count++;              // 累计完成帧数 +1
            __sync_synchronize();

            SLOG_DEBUG("HS-DONE: slot[%d] done (write_count=%d)",
                       slot, ctx->ring_write_count);

            // ★ v2.0: 搬完立即 start 下一帧 (SGDMA 提前就位等数据, 不等 frame_id)
            int next_slot = ctx->ring_write_count % RING_FRAMES;
            uint32_t status = ctx->bar2->status;
            uint16_t fid     = (uint16_t)(status & 0xFFFF);

            ctx->ring_ready[next_slot]     = 0;                    // 即将被覆盖
            ctx->ring_frame_id[next_slot]  = fid;

            // 恢复 desc.flags 为 FLAG_LAST (SGDMA 完成后会回写为 0x07)
            ctx->desc_mem[next_slot].flags = __builtin_bswap32(FLAG_LAST);
            __sync_synchronize();

            setup_c2h_channel(ctx, next_slot);
            start_sgdma_c2h(ctx);

            ctx->hs_in_flight      = 1;                            // 保持 in_flight
            ctx->hs_in_flight_slot = next_slot;
            __sync_synchronize();

            SLOG_DEBUG("HS-START: slot[%d] pre-armed (fid=%u, write_count=%d)",
                       next_slot, fid, ctx->ring_write_count);
            return 1;
        }
    }
    return 0;
}

// ============================================================================
// ★ v2.0 取显示帧 — 供渲染线程调用 (滞后 RING_DISPLAY_LAG 帧)
// ============================================================================
// 返回滞后 3 帧的 buffer 指针 (稳定, 不会被 SGDMA 覆盖), 或 NULL (帧数不足)
//
// ★ v2.0 帧头/帧尾格式 (128-bit → 内存小端字节序):
//   帧头 16B: [12..15]=magic(AA AB AC AD) [10..11]=预计行数(720) [8..9]=预计列数(1280) [6..7]=帧ID
//   帧尾 16B: [12..15]=magic(FA FB FC FD) [11]=xx(01正常/E1溢出) [10]=行状态 [9]=列状态 [7..8]=帧ID
uint8_t *sgdma_get_display_frame(SGDMAContext *ctx, uint16_t *out_frame_id) {
    if (!ctx) return NULL;

    int wc = ctx->ring_write_count;
    if (wc < RING_DISPLAY_LAG + 1)               // 至少需 4 帧完成
        return NULL;

    int base = (wc - 1 - RING_DISPLAY_LAG) % RING_FRAMES;

    // ★ v2.0: 新 magic (帧头 0xAAABACAD / 帧尾 0xFAFBFCFD)
    static const uint8_t HDR_MAGIC[4]   = {0xAA, 0xAB, 0xAC, 0xAD}; // 帧头 magic
    static const uint8_t TAIL_MAGIC[4]  = {0xFA, 0xFB, 0xFC, 0xFD}; // 帧尾 magic
    static const int    delta[5]        = {0, 1, -1, 2, -2};

    const size_t tail_base       = FRAME_TOTAL_BYTES - FRAME_TAIL_BYTES;  // 帧尾 16B 起点
    const size_t tail_magic_off  = tail_base + 12;                        // 帧尾 magic 位置
    const size_t tail_status_off = tail_base + 11;                        // 帧尾 xx 状态字节

    // 首次调用 dump base slot 帧头帧尾, 便于确认字节序 (DEBUG)
    static int header_dumped = 0;
    if (!header_dumped) {
        header_dumped = 1;
        uint8_t *b0 = ctx->buf_mem[base];
        uint8_t *t0 = ctx->buf_mem[base] + tail_base;
        SLOG_DEBUG("first header[0..15]: %02X %02X %02X %02X %02X %02X %02X %02X "
                   "%02X %02X %02X %02X %02X %02X %02X %02X",
                   b0[0], b0[1], b0[2], b0[3], b0[4], b0[5], b0[6], b0[7],
                   b0[8], b0[9], b0[10], b0[11], b0[12], b0[13], b0[14], b0[15]);
        SLOG_DEBUG("first tail[0..15]:   %02X %02X %02X %02X %02X %02X %02X %02X "
                   "%02X %02X %02X %02X %02X %02X %02X %02X",
                   t0[0], t0[1], t0[2], t0[3], t0[4], t0[5], t0[6], t0[7],
                   t0[8], t0[9], t0[10], t0[11], t0[12], t0[13], t0[14], t0[15]);
    }

    // 按优先级找第一个 帧头magic匹配 + 帧尾正常 且 ready 的完整帧
    for (int i = 0; i < 5; i++) {
        int slot = (base + delta[i] + RING_FRAMES) % RING_FRAMES;
        if (!ctx->ring_ready[slot])
            continue;
        uint8_t *buf = ctx->buf_mem[slot];

        // 帧头 magic 匹配 (错位/撕裂帧拦截)
        if (memcmp(buf + 12, HDR_MAGIC, 4) != 0)
            continue;

        // 帧尾 magic 匹配
        if (memcmp(buf + tail_magic_off, TAIL_MAGIC, 4) != 0) {
            SLOG_DEBUG("slot[%d] tail magic mismatch, skip (fallback)", slot);
            continue;
        }

        // ★ v2.0: 帧尾 xx 状态字节 = 01 正常才接受, E1 溢出丢弃
        if (buf[tail_status_off] != 0x01) {
            SLOG_DEBUG("slot[%d] frame status=0x%02X (overflow/mismatch), skip (fallback)",
                       slot, buf[tail_status_off]);
            continue;
        }

        uint16_t fid = ctx->ring_frame_id[slot];
        if (out_frame_id)
            *out_frame_id = fid;

        // ★ v2.0: -vv 每帧 dump 帧头帧尾分析 (buf 变化才 dump, 避免同帧重复)
        if (g_frame_dump) {
            static uint8_t *last_dump_buf = NULL;
            if (buf != last_dump_buf) {
                last_dump_buf = buf;
                uint8_t *h = buf;
                uint8_t *t = buf + tail_base;
                SLOG_DEBUG("FRAME[%u] HDR: magic=%02X%02X%02X%02X rows=%u cols=%u fid=%u",
                           fid,
                           h[12], h[13], h[14], h[15],
                           (unsigned)(h[10] | (h[11] << 8)),
                           (unsigned)(h[8]  | (h[9]  << 8)),
                           (unsigned)(h[6]  | (h[7]  << 8)));
                SLOG_DEBUG("FRAME[%u] TAIL: magic=%02X%02X%02X%02X status=%02X row_st=%02X col_st=%02X fid=%u",
                           fid,
                           t[12], t[13], t[14], t[15],
                           t[11], t[10], t[9],
                           (unsigned)(t[7] | (t[8] << 8)));
            }
        }

        return buf + FRAME_HEADER_BYTES;   // 跳过 16B header
    }

    // 所有候选都未就绪或错位 → 返回 NULL, 渲染线程保持上一帧
    return NULL;
}

// ============================================================================
// sgdma_init — v1.0 Handshake
// ============================================================================
int sgdma_init(SGDMAContext *ctx, const char *devnode) {
    SLOG_INFO("=== sgdma_init (v1.0 Handshake) ===");
    if (!ctx) { SLOG_ERROR("ctx is NULL"); return -1; }
    memset(ctx, 0, sizeof(*ctx));
    ctx->fd = -1;

    const char *node = devnode && *devnode ? devnode : DEFAULT_DEVNODE;

    // 1. 打开设备
    ctx->fd = open(node, O_RDWR);
    if (ctx->fd < 0) {
        SLOG_ERROR("Cannot open %s: %s", node, strerror(errno));
        return -1;
    }

    // 2. mmap BAR0 / BAR2
    ctx->bar0 = (volatile GowinBar0 *)mmap_bar(ctx->fd, 0, BAR0_SIZE);
    if (!ctx->bar0) goto fail;
    ctx->bar2 = (volatile GowinBar2 *)mmap_bar(ctx->fd, 2, BAR2_SIZE);
    if (!ctx->bar2) goto fail;
    SLOG_INFO("BAR mapped: BAR0=%p BAR2=%p", (void *)ctx->bar0, (void *)ctx->bar2);

    // 3. PCIe 初始化
    SLOG_INFO("Initializing PCIe...");
    ctx->bar0->ctrl.ctrl_init = 1;
    int timeout = 2000;
    while (ctx->bar0->ctrl.stat_init != PCIE_READY && timeout > 0 && !g_exit) {
        struct timespec ts = {.tv_sec = 0, .tv_nsec = 10000000L};
        nanosleep(&ts, NULL);
        timeout -= 10;
    }
    if (ctx->bar0->ctrl.stat_init != PCIE_READY) {
        SLOG_ERROR("PCIe not ready (stat_init=0x%08x)", ctx->bar0->ctrl.stat_init);
        goto fail;
    }
    SLOG_INFO("PCIe ready");

    // ★ v1.0: 验证 FPGA 版本号 (benchmark 前)
    uint32_t fpga_ver = ctx->bar2->version;
    SLOG_INFO("FPGA version: %d.%d (0x%08x), host version: %d.%d (0x%08x)",
              (fpga_ver >> 8) & 0xFF, fpga_ver & 0xFF, fpga_ver,
              HOST_VERSION_MAJOR, HOST_VERSION_MINOR, HOST_VERSION);
    if (fpga_ver != HOST_VERSION) {
        SLOG_ERROR("FPGA/HOST version mismatch! Expected %d.%d (0x%08x), got %d.%d (0x%08x)",
                   HOST_VERSION_MAJOR, HOST_VERSION_MINOR, HOST_VERSION,
                   (fpga_ver >> 8) & 0xFF, fpga_ver & 0xFF, fpga_ver);
        goto fail;
    }

    // 4. 分配描述符表
    ctx->desc_size = DESC_TOTAL_BYTES;
    ctx->desc_phys = request_mem(ctx->fd, 1, ctx->desc_size);
    if (!ctx->desc_phys) { SLOG_ERROR("Desc buffer allocation failed"); goto fail; }
    ctx->desc_mem = (volatile GowinDescriptor *)mmap_mem(ctx->fd, 1, ctx->desc_size);
    if (!ctx->desc_mem) goto fail;
    SLOG_INFO("Desc buffer: virt=%p phys=0x%lx", (void *)ctx->desc_mem,
              (unsigned long)ctx->desc_phys);

    // 5. 分配 5 帧环形缓冲 (每帧独立 dma_alloc_coherent, 各 3.69MB < 16MB)
    for (int i = 0; i < RING_FRAMES; i++) {
        uint64_t phys = request_mem(ctx->fd, 2 + i, FRAME_BUF_BYTES);
        if (!phys) { SLOG_ERROR("buf[%d] allocation failed", i); goto fail; }
        ctx->buf_phys[i] = phys;
        ctx->buf_mem[i] = (uint8_t *)mmap_mem(ctx->fd, 2 + i, FRAME_BUF_BYTES);
        if (!ctx->buf_mem[i]) goto fail;
        SLOG_INFO("buf[%d]: virt=%p phys=0x%lx", i,
                  (void *)ctx->buf_mem[i], (unsigned long)phys);
    }

    // 6. 设置环形描述符 (desc[i]→buf[i], 各含 FLAG_LAST)
    setup_ring_descriptors(ctx->desc_mem, ctx->desc_phys, ctx->buf_phys, FRAME_TOTAL_BYTES);
    // poll 字清零
    memset((void *)((uint8_t *)ctx->desc_mem + DESC_TOTAL_BYTES - 64), 0, 64);

    // 7. ★ v1.1.x: 先 stream_reset 复位 C2H 数据通路, 再武装 host_ready
    //   复位对象: CDC FIFO 写侧/读侧 + frame_header_inserter + axis_width_up
    //   目的: 清空 FIFO + 让 header_inserter 回 IDLE 重新从干净帧头插 magic,
    //         修 host 重启时"gate 还开着、SGDMA 从像素流中间开始搬"导致的帧头/帧尾丢失
    SLOG_INFO("Resetting C2H data path (stream_reset=1)...");
    ctx->bar2->stream_reset = 1;        // bit0=1 复位 (电平信号, 保持)
    __sync_synchronize();
    usleep(2000);                       // 保持复位 2ms (cam_pclk 72MHz / tlp_clk 100MHz 均足够)
    ctx->bar2->stream_reset = 0;        // bit0=0 释放
    __sync_synchronize();
    (void)ctx->bar2->status;            // PCIe 屏障
    usleep(1000);                       // 等 CDC FIFO 指针清零稳定
    SLOG_INFO("C2H data path released (stream_reset=0)");

    // 8. ★ v2.0: 先 start SGDMA 第一帧 (提前就位等数据), 再写 host_ready=1
    //   SGDMA 启动后 FIFO 空 → 等 vsync↓ 数据从干净帧头流入立即搬 (消除启动延迟)
    setup_c2h_channel(ctx, 0);
    start_sgdma_c2h(ctx);
    ctx->hs_in_flight      = 1;
    ctx->hs_in_flight_slot = 0;
    ctx->ring_ready[0]     = 0;
    __sync_synchronize();
    SLOG_DEBUG("HS-START: slot[0] pre-armed (SGDMA waiting for data)");

    // 9. ★ v2.0: 写 host_ready=1 → FPGA vsync↓ 开门 (FIFO 刚被 vsync 高清空)
    ctx->bar2->ctrl = BAR2_CTRL_HOST_READY;
    __sync_synchronize();
    (void)ctx->bar2->status;   // PCIe 屏障

    ctx->last_frame_id = sgdma_frame_id(ctx);
    SLOG_INFO("Initial frame_id=%u, vsync=%d, SGDMA pre-armed + host_ready=1 (gate opens at vsync↓)",
              ctx->last_frame_id, sgdma_vsync_state(ctx));
    SLOG_INFO("sgdma_init OK");
    return 0;

fail:
    SLOG_ERROR("sgdma_init FAILED");
    sgdma_cleanup(ctx);
    return -1;
}

// ============================================================================
// sgdma_cleanup
// ============================================================================
void sgdma_cleanup(SGDMAContext *ctx) {
    if (!ctx) return;
    SLOG_DEBUG("=== sgdma_cleanup ===");

    // ★ v1.1.2: 复位 host_ready=0 → FPGA gate 直接关门 (下次启动从帧尾重新对齐)
    if (ctx->bar2) {
        ctx->bar2->ctrl = 0;
        __sync_synchronize();
        (void)ctx->bar2->status;   // PCIe 屏障
    }

    if (ctx->desc_mem) { munmap((void *)ctx->desc_mem, ctx->desc_size); ctx->desc_mem = NULL; }
    if (ctx->desc_phys) { release_mem(ctx->fd, 1); ctx->desc_phys = 0; }

    for (int i = 0; i < RING_FRAMES; i++) {
        if (ctx->buf_mem[i]) { munmap(ctx->buf_mem[i], FRAME_BUF_BYTES); ctx->buf_mem[i] = NULL; }
        if (ctx->buf_phys[i]) { release_mem(ctx->fd, 2 + i); ctx->buf_phys[i] = 0; }
    }

    if (ctx->bar2) { munmap((void *)ctx->bar2, BAR2_SIZE); ctx->bar2 = NULL; }
    if (ctx->bar0) { munmap((void *)ctx->bar0, BAR0_SIZE); ctx->bar0 = NULL; }
    if (ctx->fd >= 0) { close(ctx->fd); ctx->fd = -1; }
    SLOG_DEBUG("Cleanup complete");
}

// ============================================================================
// ★ sgdma_benchmark_bar2_latency — BAR2 读写延迟测量
// ============================================================================
int sgdma_benchmark_bar2_latency(SGDMAContext *ctx) {
    SLOG_INFO("=== BAR2 读写延迟基准 (100 次采样) ===");

    #define LATENCY_SAMPLES 100

    double read_total_ns = 0.0;
    double write_total_ns = 0.0;
    struct timespec t1, t2;

    // ---- BAR2 读延迟 ----
    for (int i = 0; i < LATENCY_SAMPLES; i++) {
        clock_gettime(CLOCK_MONOTONIC, &t1);
        volatile uint32_t dummy = ctx->bar2->status;  // BAR2 读
        clock_gettime(CLOCK_MONOTONIC, &t2);
        (void)dummy;
        read_total_ns += (t2.tv_sec - t1.tv_sec) * 1e9 + (t2.tv_nsec - t1.tv_nsec);
    }

    // ---- BAR2 写延迟 ----
    // ★ v1.4: 翻转保留 bit31 (logic_dma 忽略), 不干扰 host_ready(bit4) 武装状态
    uint32_t ctrl_save = ctx->bar2->ctrl;
    for (int i = 0; i < LATENCY_SAMPLES; i++) {
        uint32_t val = ctrl_save ^ (1u << 31);   // 翻转保留位, 仅测写延迟
        clock_gettime(CLOCK_MONOTONIC, &t1);
        ctx->bar2->ctrl = val;           // BAR2 写
        __sync_synchronize();
        (void)ctx->bar2->status;          // PCIe 屏障
        clock_gettime(CLOCK_MONOTONIC, &t2);
        write_total_ns += (t2.tv_sec - t1.tv_sec) * 1e9 + (t2.tv_nsec - t1.tv_nsec);
    }
    // 恢复
    ctx->bar2->ctrl = ctrl_save;
    __sync_synchronize();
    (void)ctx->bar2->status;

    ctx->bar2_read_us  = read_total_ns  / LATENCY_SAMPLES / 1000.0;
    ctx->bar2_write_us = write_total_ns / LATENCY_SAMPLES / 1000.0;

    // 估算握手总时间
    double handshake_est = ctx->bar2_read_us + ctx->bar2_write_us * 2 + 5.0;  // +5μs 描述符/逻辑开销
    SLOG_INFO("  BAR2 读:      %.2f μs", ctx->bar2_read_us);
    SLOG_INFO("  BAR2 写:      %.2f μs", ctx->bar2_write_us);
    SLOG_INFO("  握手估算:     %.2f μs (读 + 2×写 + 逻辑)", handshake_est);
    SLOG_INFO("  消隐窗口 550μs → 余量: %.0f×",
              550.0 / handshake_est);
    return 0;
}

// ============================================================================
// BMP 帧保存 (-vvb 调试用)
// ============================================================================
// BMP 格式: BITMAPFILEHEADER(14) + BITMAPINFOHEADER(40) + BGRA pixels
// 保存路径: ./frames/frame_XXXXXX.bmp (frame_id 补零到 6 位)
// ============================================================================

int sgdma_save_frame_bmp(const uint8_t *pixels, uint16_t frame_id,
                          int width, int height) {
    if (!pixels) return -1;

    // 创建 frames 目录
    static int dir_created = 0;
    if (!dir_created) {
        mkdir("frames", 0755);
        dir_created = 1;
    }

    char path[128];
    snprintf(path, sizeof(path), "frames/frame_%06u.bmp", frame_id);

    FILE *fp = fopen(path, "wb");
    if (!fp) {
        SLOG_WARN("Cannot create BMP: %s", path);
        return -1;
    }

    uint32_t row_size = width * 4;  // BGRA = 4 bytes/pixel
    uint32_t img_size = row_size * height;
    uint32_t file_size = 54 + img_size;

    // BITMAPFILEHEADER (14 bytes)
    uint8_t fh[14] = {0};
    fh[0] = 'B'; fh[1] = 'M';                          // bfType
    fh[2] = (file_size >>  0) & 0xFF;                  // bfSize
    fh[3] = (file_size >>  8) & 0xFF;
    fh[4] = (file_size >> 16) & 0xFF;
    fh[5] = (file_size >> 24) & 0xFF;
    fh[10] = 54;                                        // bfOffBits

    // BITMAPINFOHEADER (40 bytes)
    uint8_t ih[40] = {0};
    ih[0] = 40;                                          // biSize
    ih[4] = (width  >>  0) & 0xFF;                      // biWidth
    ih[5] = (width  >>  8) & 0xFF;
    ih[6] = (width  >> 16) & 0xFF;
    ih[7] = (width  >> 24) & 0xFF;
    ih[8] = (height >>  0) & 0xFF;                      // biHeight
    ih[9] = (height >>  8) & 0xFF;
    ih[10]= (height >> 16) & 0xFF;
    ih[11]= (height >> 24) & 0xFF;
    ih[12]= 1;                                           // biPlanes
    ih[14]= 32;                                          // biBitCount

    fwrite(fh, 1, 14, fp);
    fwrite(ih, 1, 40, fp);

    // BMP bottom-up: write scanlines in reverse
    for (int y = height - 1; y >= 0; y--) {
        fwrite(pixels + y * row_size, 1, row_size, fp);
    }

    fclose(fp);
    SLOG_DEBUG("BMP saved: %s (%dx%d, %u bytes)", path, width, height, file_size);
    return 0;
}
