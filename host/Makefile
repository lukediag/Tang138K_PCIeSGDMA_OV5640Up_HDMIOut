# ============================================================================
# Makefile — SGDMADDR5v0.3 Host 实时视频采集 GUI (DDR3 Bypass + Streaming C2H)
# ============================================================================
# 目标:
#   make              — 编译 video_gui (ping-pong + 零拷贝 BGRA + 日志)
#   make driver       — 编译内核驱动 gowin_demo.ko
#   make all          — 编译所有 (应用 + 驱动)
#   make clean        — 清理全部
#
# 依赖:
#   apt install libsdl2-dev libsdl2-ttf-dev
#
# 快速开始:
#   make all
#   sudo insmod bin/gowin_demo.ko
#   sudo ./bin/video_gui          # INFO 级别
#   sudo ./bin/video_gui -v       # DEBUG 级别
#   sudo ./bin/video_gui -vv      # TRACE 级别
#
# 交叉编译:
#   make CROSS_COMPILE=aarch64-linux-gnu-
# ============================================================================

SHELL = /bin/bash

# ---- 用户态应用 ----
CC      ?= gcc
TARGET   = video_gui

SRCS     = main.c utils/sgdma_core.c utils/log.c
OBJS     = $(SRCS:.c=.o)

BIN_DIR  = bin
DRV_DIR  = driver

# ---- SDL2 配置 ----
SDL2_CFLAGS  ?= $(shell pkg-config --cflags sdl2 SDL2_ttf 2>/dev/null)
SDL2_LIBS    ?= $(shell pkg-config --libs   sdl2 SDL2_ttf 2>/dev/null)

ifeq ($(SDL2_LIBS),)
    $(warning ! pkg-config 未找到 sdl2/SDL2_ttf, 尝试手动路径)
    SDL2_CFLAGS = -I/usr/include/SDL2
    SDL2_LIBS   = -lSDL2 -lSDL2_ttf
endif

CFLAGS  += -std=c11 -Wall -O2 -pthread -I. -Iutils -Iinclude $(SDL2_CFLAGS)
LDFLAGS += $(SDL2_LIBS) -lm -pthread

ifneq ($(CROSS_COMPILE),)
    CC = $(CROSS_COMPILE)gcc
endif

.PHONY: all apps driver clean clean-apps clean-driver help

all: apps driver

apps: $(BIN_DIR)/$(TARGET)

driver:
	@echo "===== 编译内核驱动 ====="
	$(MAKE) -C $(DRV_DIR)
	@mkdir -p $(BIN_DIR)
	@cp -f $(DRV_DIR)/gowin_demo.ko $(BIN_DIR)/ 2>/dev/null || true
	@echo "  → $(BIN_DIR)/gowin_demo.ko"

$(BIN_DIR)/$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)
	@echo "  → 编译完成: $@"

$(BIN_DIR):
	@mkdir -p $(BIN_DIR)

main.o: main.c utils/sgdma_core.h utils/log.h
	$(CC) $(CFLAGS) -c -o $@ $<

utils/sgdma_core.o: utils/sgdma_core.c utils/sgdma_core.h utils/log.h
	$(CC) $(CFLAGS) -c -o $@ $<

utils/log.o: utils/log.c utils/log.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean: clean-apps clean-driver
	@echo "  → 全部清理完成"

clean-apps:
	@rm -rf $(BIN_DIR)/$(TARGET) main.o utils/sgdma_core.o utils/log.o

clean-driver:
	$(MAKE) -C $(DRV_DIR) clean
	@rm -f $(BIN_DIR)/gowin_demo.ko

help:
	@echo "SGDMADDR4v0 Host — ping-pong 实时视频采集 GUI"
	@echo ""
	@echo "  make              编译应用 (video_gui)"
	@echo "  make driver       编译内核驱动 (gowin_demo.ko)"
	@echo "  make all          编译所有 (应用 + 驱动)"
	@echo "  make clean        清理全部"
	@echo ""
	@echo "  CROSS_COMPILE=     交叉编译前缀 (如 aarch64-linux-gnu-)"
	@echo ""
	@echo "运行:"
	@echo "  sudo insmod bin/gowin_demo.ko   # 加载驱动"
	@echo "  sudo ./bin/video_gui            # 启动 GUI (INFO)"
	@echo "  sudo ./bin/video_gui -v         # 启动 GUI (DEBUG)"
	@echo "  sudo ./bin/video_gui -vv        # 启动 GUI (TRACE)"
	@echo ""
	@echo "日志: ./logs/sgdma_gui_<timestamp>.log"
