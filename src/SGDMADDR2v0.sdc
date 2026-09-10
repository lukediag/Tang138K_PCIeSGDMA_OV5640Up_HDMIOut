//Copyright (C)2014-2024 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.10.02
//Created Time: 2024-09-10 19:06:01

// ============================================================================
// 25MHz 系统晶振输入
// ============================================================================
create_clock -name clk_25 -period 40 -waveform {0 20} [get_ports {sys_clkin}]

// ============================================================================
// HDMI 时钟域 (Gowin_PLL_HDMI: VCO=750MHz)
// ============================================================================
create_clock -name pixel_clk -period 13.3333 -waveform {0 6.6667} [get_nets {pixel_clk}]
create_clock -name pixel_clk_5x -period 2.6667 -waveform {0 1.3333} [get_nets {pixel_clk_5x}]

// ============================================================================
// OV5640 像素时钟 (cam_pclk, 72MHz 外部输入, ★ v1.1 修复: 原缺失导致采集链路时序未约束)
// ============================================================================
create_clock -name cam_pclk -period 13.888 -waveform {0 6.944} [get_ports {cam_pclk}]

// ============================================================================
// 100MHz 系统时钟 (CLKDIV: 200MHz ÷ 2)
// ============================================================================
create_clock -name div_clk -period 10 -waveform {0 5} [get_pins {uut_div2/CLKOUT}]

// ============================================================================
// v5.0.3: DDR3 已删除, 无 ui_clk/ddr_out_clk 时钟域
// ============================================================================

// ============================================================================
// 异步时钟组 (4 个独立时钟域)
//   div_clk     — 100MHz PCIe (tlp_clk), C2H 数据路径
//   pixel_clk   — 75MHz HDMI 像素
//   pixel_clk_5x — 375MHz TMDS 串行
//   cam_pclk    — 72MHz OV5640 采集 (★ v1.1 新增)
// ============================================================================
set_clock_groups -asynchronous -group [get_clocks {div_clk}] -group [get_clocks {pixel_clk}] -group [get_clocks {pixel_clk_5x}] -group [get_clocks {cam_pclk}]
