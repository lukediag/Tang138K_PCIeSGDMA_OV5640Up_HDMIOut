// ============================================================================
// gowin_bar2.h — SGDMADDR5v0.3 BAR2 寄存器映射 (DDR3 Bypass)
// ============================================================================
// 基于 logic_dma.v v5.0.3 寄存器定义
// v5.0.3 删除了 C2H/LAD/Video 寄存器组, 仅保留 Ctrl/Status/H2C
//
// BAR2 空间: 0x000 ~ 0x7FF (2048 bytes)
//
// 寄存器布局:
//   0x00  Ctrl           [RW]  bit0=H2C_start, bit1=H2C_stop
//   0x04  Status         [RO]  bit0=H2C_valid, [15:0]=frame_count
//   0x10  AddrDDRh2c     [RW]  H2C DDR 目标地址 (保留, H2C 路径 terminated)
//   0x14  LengDDRh2c     [RW]  H2C 传输长度
//   0x18  Overheadh2cLo  [RO]  H2C overhead 低 32-bit
//   0x1C  Overheadh2cHi  [RO]  H2C overhead 高 32-bit
// ============================================================================

#ifndef GOWIN_BAR2_H
#define GOWIN_BAR2_H

#include <stdint.h>

#define BAR2_SIZE (1024 * 2)

// ---- BAR2 Ctrl 寄存器位定义 (0x00) ----
#define BAR2_PCIE_WR_START  (1 << 0)   // H2C 启动
#define BAR2_PCIE_WR_STOP   (1 << 1)   // H2C 停止

// ---- BAR2 Status 寄存器位定义 (0x04) ----
#define BAR2_STATUS_H2C_VALID    (1 << 0)       // H2C valid
#define BAR2_STATUS_FRAME_COUNT(s) ((s) & 0xFFFF) // [15:0] = frame_count

// ============================================================================
// BAR2 寄存器结构体 (mmap 映射) — v5.0.3 精简版
// ============================================================================
typedef struct __attribute__((packed, aligned(32))) {
    // ---- 控制 & 状态 ----
    volatile uint32_t ctrl;             // 0x00 - 控制寄存器 (RW)
    volatile uint32_t status;           // 0x04 - 状态寄存器 (RO), [15:0]=frame_count
    volatile uint32_t rsv_08[2];        // 0x08-0x0F

    // ---- H2C (Host → DDR) — 保留供未来控制通道 ----
    volatile uint32_t addr_ddr_h2c;     // 0x10 - H2C DDR 地址 (RW)
    volatile uint32_t leng_ddr_h2c;     // 0x14 - H2C 长度 (RW)
    volatile uint32_t rsv_18[2];        // 0x18-0x1F (overhead 只读)

    // ---- 0x20-0x7FF: 旧寄存器已删除 (C2H/LAD/Video) ----
    volatile uint32_t rsv_20[492];      // padding 到 2048 bytes
} GowinBar2;

// ---- 编译时结构体大小校验 ----
_Static_assert(sizeof(GowinBar2) == BAR2_SIZE,
               "GowinBar2 size mismatch: must be 2048 bytes");

#endif // GOWIN_BAR2_H
