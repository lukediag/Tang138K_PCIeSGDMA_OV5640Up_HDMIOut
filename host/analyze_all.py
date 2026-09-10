# -*- coding: utf-8 -*-
"""批量分析所有 BMP 帧: 检测彩条边界偏移 / 撕裂 / 异常颜色 (纯标准库)"""
import struct, sys, os, collections

def read_bmp(path):
    with open(path, 'rb') as f:
        data = f.read()
    off = struct.unpack('<I', data[10:14])[0]
    w = struct.unpack('<i', data[18:22])[0]
    h = struct.unpack('<i', data[22:26])[0]
    return w, h, data[off:]

def px(pxd, w, x, y):
    off = y * w * 4 + x * 4
    return (pxd[off+2], pxd[off+1], pxd[off])  # RGB

def row_boundaries(pxd, w, y):
    """返回该行 y 的颜色边界 x 坐标列表"""
    prev = None
    bs = []
    for x in range(w):
        c = px(pxd, w, x, y)
        if prev is not None and c != prev:
            bs.append(x)
        prev = c
    return bs

def main():
    folder = 'frames'
    files = sorted(f for f in os.listdir(folder) if f.endswith('.bmp'))
    print(f"共 {len(files)} 帧, 预期彩条边界 = [256, 512, 768, 1024]\n")
    for fn in files:
        w, h, pxd = read_bmp(os.path.join(folder, fn))
        # 采样 5 行 (底部/下中/中/上中/顶部), 找各自边界
        ys = [10, h//4, h//2, h*3//4, h-10]
        boundary_sets = []
        ok = True
        for y in ys:
            bs = row_boundaries(pxd, w, y)
            boundary_sets.append(bs)
            if bs != [256, 512, 768, 1024]:
                ok = False
        # 行内颜色序列 (y=中间)
        mid = row_boundaries(pxd, w, h//2)
        # 判断是否撕裂: 不同行边界不同
        tearing = len(set(map(tuple, boundary_sets))) > 1
        # 颜色检查: 采样每个条带中点
        colors = [px(pxd, w, 128, h//2), px(pxd, w, 384, h//2),
                  px(pxd, w, 640, h//2), px(pxd, w, 896, h//2),
                  px(pxd, w, 1152, h//2)]
        status = "OK " if ok else "异常"
        if tearing:
            status += " [撕裂]"
        print(f"{fn}: {status}  边界(各行)={boundary_sets[2]}  条带色={colors}")
        if not ok:
            for y, bs in zip(ys, boundary_sets):
                print(f"      y={y}: {bs}")

if __name__ == '__main__':
    main()
