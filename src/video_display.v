//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           video_display
// Created by:          正点原子
// Created date:        2025年10月14日15:07:00
// Version:             V2.2
// Descriptions:        视频显示模块 — 动态测试图案 (每秒切换)
//
//  v2.2 改动: 由静态彩条改为 6 种图案每秒轮换:
//    pattern 0: 垂直彩条 (8 条)
//    pattern 1: 水平彩条 (6 条)
//    pattern 2: 纯红
//    pattern 3: 纯白
//    pattern 4: 纯蓝
//    pattern 5: 色块棋盘格 (64px, 4 色)
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module  video_display(
    input                pixel_clk,
    input                sys_rst_n,
    
    input        [10:0]  pixel_xpos,  //像素点横坐标
    input        [10:0]  pixel_ypos,  //像素点纵坐标
    output  reg  [23:0]  pixel_data   //像素点数据
);

//parameter define
parameter  H_DISP = 11'd1280;                        //分辨率——行
parameter  V_DISP = 11'd720;                         //分辨率——列

localparam WHITE   = 24'hFFFFFF;                     //RGB888 白色
localparam YELLOW  = 24'hFFFF00;                     //RGB888 黄色
localparam CYAN    = 24'h00FFFF;                     //RGB888 青色
localparam GREEN   = 24'h00FF00;                     //RGB888 绿色
localparam MAGENTA = 24'hFF00FF;                     //RGB888 品红
localparam RED     = 24'hFF0000;                     //RGB888 红色
localparam BLUE    = 24'h0000FF;                     //RGB888 蓝色
localparam BLACK   = 24'h000000;                     //RGB888 黑色

// ★ v2.2: 每秒切换图案计时 (pixel_clk = 75MHz → 1 秒 = 75,000,000 拍)
localparam [26:0] ONE_SEC = 27'd75_000_000 - 27'd1;

reg [26:0] time_cnt;    // 秒计时器
reg [ 2:0] pattern_sel; // 图案序号 0..5
    
//*****************************************************
//**                    main code
//*****************************************************

// ★ v2.2: 每秒切换一次图案 (0→1→2→3→4→5→0 循环)
always @(posedge pixel_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        time_cnt    <= 27'd0;
        pattern_sel <= 3'd0;
    end else if (time_cnt == ONE_SEC) begin
        time_cnt    <= 27'd0;
        pattern_sel <= (pattern_sel == 3'd5) ? 3'd0 : pattern_sel + 3'd1;
    end else begin
        time_cnt    <= time_cnt + 27'd1;
    end
end

//根据当前图案 + 像素坐标生成像素颜色
always @(posedge pixel_clk ) begin
    if (!sys_rst_n)
        pixel_data <= 24'd0;
    else begin
        case (pattern_sel)
            // ---- 图案 0: 垂直彩条 (8 条, 每条 160px) ----
            3'd0: begin
                if      (pixel_xpos < 11'd160)  pixel_data <= WHITE;
                else if (pixel_xpos < 11'd320)  pixel_data <= YELLOW;
                else if (pixel_xpos < 11'd480)  pixel_data <= CYAN;
                else if (pixel_xpos < 11'd640)  pixel_data <= GREEN;
                else if (pixel_xpos < 11'd800)  pixel_data <= MAGENTA;
                else if (pixel_xpos < 11'd960)  pixel_data <= RED;
                else if (pixel_xpos < 11'd1120) pixel_data <= BLUE;
                else                            pixel_data <= BLACK;
            end

            // ---- 图案 1: 水平彩条 (6 条, 每条 120px) ----
            3'd1: begin
                if      (pixel_ypos < 11'd120)  pixel_data <= WHITE;
                else if (pixel_ypos < 11'd240)  pixel_data <= YELLOW;
                else if (pixel_ypos < 11'd360)  pixel_data <= CYAN;
                else if (pixel_ypos < 11'd480)  pixel_data <= GREEN;
                else if (pixel_ypos < 11'd600)  pixel_data <= MAGENTA;
                else                            pixel_data <= RED;
            end

            // ---- 图案 2: 纯红 ----
            3'd2: pixel_data <= RED;

            // ---- 图案 3: 纯白 ----
            3'd3: pixel_data <= WHITE;

            // ---- 图案 4: 纯蓝 ----
            3'd4: pixel_data <= BLUE;

            // ---- 图案 5: 色块棋盘格 (64px × 64px, 4 色) ----
            3'd5: begin
                case ({pixel_ypos[6], pixel_xpos[6]})
                    2'b00: pixel_data <= RED;
                    2'b01: pixel_data <= GREEN;
                    2'b10: pixel_data <= BLUE;
                    2'b11: pixel_data <= YELLOW;
                    default: pixel_data <= BLACK;
                endcase
            end

            default: pixel_data <= 24'd0;
        endcase
    end
end

endmodule