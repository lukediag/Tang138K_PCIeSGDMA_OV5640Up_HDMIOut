# ============================================================================
# Makefile — Gowin PCIe Demo 内核驱动
# ============================================================================
# 编译:
#   make -C ../ driver
# 或在本目录:
#   make
#
# 加载/卸载:
#   sudo insmod gowin_demo.ko
#   sudo rmmod gowin_demo
# ============================================================================

MODULE_NAME  = gowin_demo
obj-m       += $(MODULE_NAME).o
$(MODULE_NAME)-objs += gowin_demo_drv.o

# 内核源码目录 (可覆盖)
KERNEL_DIR  ?= /lib/modules/$(shell uname -r)/build

# 本目录 (传给内核构建系统作为 M= 参数)
SRC_DIR     ?= $(shell pwd)

.PHONY: all modules clean

all: modules

modules:
	@echo "===== 编译 $(MODULE_NAME).ko ====="
	$(MAKE) -C $(KERNEL_DIR) M=$(SRC_DIR) modules
	@strip --strip-debug $(MODULE_NAME).ko 2>/dev/null || true
	@echo "  → $(MODULE_NAME).ko 编译完成"

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(SRC_DIR) clean
	@rm -f Module.symvers modules.order *.o *.ko *.mod.c .*.cmd
