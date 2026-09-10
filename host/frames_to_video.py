#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将 frames/ 目录下的 BMP 帧序列拼接成视频。

用法（在 host/ 目录下运行）:
    python3 frames_to_video.py [fps] [输出文件]
    默认: fps=27, 输出 video.mp4

依赖: opencv-python
    pip install opencv-python   (板子/PC 均可)
"""
import sys
import glob

try:
    import cv2
except ImportError:
    print("缺少 opencv，请先安装: pip install opencv-python")
    sys.exit(1)


def main():
    fps = float(sys.argv[1]) if len(sys.argv) > 1 else 27.0
    out = sys.argv[2] if len(sys.argv) > 2 else "video.mp4"

    files = sorted(glob.glob("frames/frame_*.bmp"))
    if not files:
        print("没找到 frames/frame_*.bmp，请在 host/ 目录下运行")
        return 1

    # 用第一帧确定尺寸
    first = cv2.imread(files[0], cv2.IMREAD_UNCHANGED)
    if first is None:
        print(f"读第一帧失败: {files[0]}")
        return 1
    h, w = first.shape[:2]
    print(f"帧数: {len(files)} | 尺寸: {w}x{h} | fps: {fps}")

    # 编码器: 优先 mp4v，失败回退 avi (MJPG)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    vw = cv2.VideoWriter(out, fourcc, fps, (w, h))
    if not vw.isOpened():
        out = "video.avi"
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        vw = cv2.VideoWriter(out, fourcc, fps, (w, h))
    if not vw.isOpened():
        print("无法创建视频文件，请检查 opencv 是否带编码支持")
        return 1

    n = 0
    for f in files:
        img = cv2.imread(f, cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        # BGRA(4通道) → BGR(3通道)，编码器只认 3 通道
        if img.ndim == 3 and img.shape[2] == 4:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if img.shape[:2] != (h, w):
            img = cv2.resize(img, (w, h))
        vw.write(img)
        n += 1

    vw.release()
    print(f"完成: {n} 帧 → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
