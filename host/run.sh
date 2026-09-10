#!/bin/bash
# ============================================================================
# run.sh — sgdmaVideoShakeHand 一键启动 (环形 5 帧 + BMP dump)
# ============================================================================
# 用法:
#   ./run.sh             默认 INFO 级别
#   ./run.sh -v          DEBUG 级别
#   ./run.sh -vv         TRACE 级别
#   ./run.sh -vvb        TRACE + 保存 BMP 到 ./frames/
#   ./run.sh -headless   纯采集无显示 (每 10 帧打印 FPS)
#
# 显示自动检测 (非 headless):
#   - 物理显示器 + 新 SDL 2.28(/usr/local/lib) → KMSDRM 直接输出 HDMI (硬件 vsync 无撕裂)
#     自动停 lightdm, 退出自动恢复
#   - 物理显示器 + 旧 SDL(2.0.20) → x11:0 输出到物理显示器
#   - 无物理显示器 → 回退 VNC :2 (X11)
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "===== SGDMADDR5v0.4 Ring Streaming ====="
echo ""

# ---- Step 1: Clean build ----
echo "===== Step 1: Clean build ====="
make clean
make all

# ---- Step 2: Create directories ----
echo ""
echo "===== Step 2: Create directories ====="
mkdir -p logs frames
echo "  -> logs/ and frames/ ready"

# ---- Step 3: Remove old driver ----
echo ""
echo "===== Step 3: Remove old driver ====="
sudo rmmod gowin_demo 2>/dev/null || true
echo "  -> old driver removed (if any)"

# ---- Step 4: Insert new driver ----
echo ""
echo "===== Step 4: Insert new driver ====="
sudo insmod bin/gowin_demo.ko
echo "  -> driver loaded"

# ---- Step 5: Device permissions ----
echo ""
echo "===== Step 5: Set device permissions ====="
sudo chmod 666 /dev/gowin_pcie_demo

# ---- Step 6: Auto-detect display backend ----
echo ""
echo "===== Step 6: Auto-detect display ====="

# 判断是否 headless (纯采集, 无需任何显示)
HEADLESS=0
for a in "$@"; do
    [ "$a" = "-headless" ] && HEADLESS=1
done

# 检测物理显示器 (DRM connector 状态: connected=已接显示器)
PHYSICAL=0
for f in /sys/class/drm/card*-*/status; do
    if [ -f "$f" ] && [ "$(cat "$f" 2>/dev/null)" = "connected" ]; then
        PHYSICAL=1
    fi
done

# 检测新 SDL 2.28+ (自编译, 修复了 KMSDRM libudev bug, 装在 /usr/local/lib)
NEW_SDL=0
[ -f /usr/local/lib/libSDL2-2.0.so.0 ] && NEW_SDL=1

# 检测本地 X server :0 (板子接显示器且跑桌面环境时存在)
LOCAL_X=0
[ -S /tmp/.X11-unix/X0 ] && LOCAL_X=1

echo "  DRM devices: $(ls /dev/dri/ 2>/dev/null | tr '\n' ' ' || true)"
echo "  Physical display: $([ "$PHYSICAL" = "1" ] && echo DETECTED || echo none)"
echo "  New SDL (KMSDRM fix): $([ "$NEW_SDL" = "1" ] && echo yes || echo no)"
echo "  Local X server :0: $([ "$LOCAL_X" = "1" ] && echo yes || echo no)"

# 退出时恢复桌面 (KMSDRM 分支会停 lightdm)
RESTORE_LIGHTDM=0
restore_lightdm() {
    if [ "$RESTORE_LIGHTDM" = "1" ]; then
        echo ""
        echo "  -> restoring lightdm (desktop)..."
        sudo systemctl start lightdm 2>/dev/null || true
    fi
}

if [ "$HEADLESS" = "1" ]; then
    echo "  -headless mode: skip display setup"
    unset DISPLAY
elif [ "$PHYSICAL" = "1" ] && [ "$NEW_SDL" = "1" ]; then
    # 物理显示器 + 新 SDL → KMSDRM 直接输出到 HDMI (DRM page flip 硬件 vsync, 无撕裂)
    echo "  -> KMSDRM (HDMI direct, hardware vsync)"
    export SDL_VIDEODRIVER=KMSDRM
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    mkdir -p "$XDG_RUNTIME_DIR"
    if systemctl is-active --quiet lightdm 2>/dev/null; then
        echo "  -> stopping lightdm (KMSDRM needs DRM master)..."
        sudo systemctl stop lightdm
        RESTORE_LIGHTDM=1
        trap restore_lightdm EXIT
    fi
elif [ "$PHYSICAL" = "1" ] && [ "$LOCAL_X" = "1" ]; then
    # 物理显示器 + 本地 X (旧 SDL 2.0.20, 无 KMSDRM) → x11:0
    echo "  -> Local X server :0 (output to physical display)"
    export SDL_VIDEODRIVER=x11
    export DISPLAY=:0
else
    # 其余情况 (无物理显示器, 或物理显示器但无本地 X) → VNC :2
    echo "  -> VNC :2 (X11)"
    export SDL_VIDEODRIVER=x11
    export DISPLAY=:2
fi
echo "  SDL_VIDEODRIVER=${SDL_VIDEODRIVER:-default}  DISPLAY=${DISPLAY:-<unset>}"

# ---- Step 7: Launch ----
echo ""
echo "===== Step 7: Launch GUI ====="
echo "  Args: $@"
echo ""

sudo -E ./bin/video_gui "$@"

echo ""
echo "===== Done ====="
echo "  Log: $(ls -t logs/sgdma_gui_*.log 2>/dev/null | head -1 || echo 'N/A')"
