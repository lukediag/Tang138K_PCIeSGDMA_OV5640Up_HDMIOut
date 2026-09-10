# -*- coding: utf-8 -*-
"""分析 video_gui 保存的 BMP 帧, 诊断画面混乱原因 (纯标准库, 无需 numpy)"""
import struct, sys, collections, os

def read_bmp(path):
    with open(path, 'rb') as f:
        data = f.read()
    assert data[0:2] == b'BM', "not a BMP"
    data_offset = struct.unpack('<I', data[10:14])[0]
    width  = struct.unpack('<i', data[18:22])[0]
    height = struct.unpack('<i', data[22:26])[0]
    bpp    = struct.unpack('<H', data[28:30])[0]
    return width, height, bpp, data_offset, data

def pixel_at(px, w, x, y_bottom_up):
    off = y_bottom_up * w * 4 + x * 4
    b, g, r = px[off], px[off+1], px[off+2]
    return (r, g, b)

def analyze(path):
    w, h, bpp, off, data = read_bmp(path)
    px = data[off:]
    print(f"=== {os.path.basename(path)}: {w}x{h} bpp={bpp} ===")

    # 1) 颜色直方图 (采样)
    cnt = collections.Counter()
    for y in range(0, h, 4):
        for x in range(0, w, 4):
            cnt[pixel_at(px, w, x, y)] += 1
    print("颜色直方图 Top12 (RGB):")
    for c, n in cnt.most_common(12):
        print(f"  #{c[0]:02x}{c[1]:02x}{c[2]:02x}  x{n}")

    # 2) 中间一行 (bottom-up y=360) 前 300 像素颜色序列
    print("\n行 y=360 前 300 像素 (每 20 换行):")
    line = []
    for x in range(300):
        r, g, b = pixel_at(px, w, x, 360)
        line.append(f"{r:02x}{g:02x}{b:02x}")
    for i in range(0, 300, 20):
        print("  " + " ".join(line[i:i+20]))

    # 3) 检查同一列不同行的颜色是否一致 (垂直一致性)
    print("\n垂直一致性检查 (x=100/400/700/1000 列, 每 50 行采样):")
    for x in [100, 400, 700, 1000]:
        cols = []
        for y in range(0, h, 50):
            cols.append(pixel_at(px, w, x, y))
        uniq = set(cols)
        print(f"  x={x}: {len(uniq)} 种颜色, 序列={cols[:8]}")

    # 4) 检查相邻帧间的列模式 (彩条边界)
    print("\n彩条边界检测 (y=360, 找颜色变化点 x):")
    prev = None
    boundaries = []
    for x in range(w):
        c = pixel_at(px, w, x, 360)
        if prev is not None and c != prev:
            boundaries.append((x, prev, c))
        prev = c
    print(f"  共 {len(boundaries)} 个颜色边界")
    for x, p, c in boundaries[:40]:
        print(f"    x={x}: {p[0]:02x}{p[1]:02x}{p[2]:02x} -> {c[0]:02x}{c[1]:02x}{c[2]:02x}")

if __name__ == '__main__':
    paths = sys.argv[1:] or sorted(os.listdir('frames'))
    if not os.path.sep in (paths[0] if paths else ''):
        paths = [os.path.join('frames', p) for p in paths]
    for p in paths:
        try:
            analyze(p)
            print()
        except Exception as e:
            print(f"!! {p}: {e}")
