#!/usr/bin/env python3
"""Decode verification for the 640x480 rolling color bar xk264 AXI sim."""
import subprocess, sys, math, os

FFMPEG = "/home/kw/codePrj/axi_xk264/tools/ffmpeg-master-latest-linux64-gpl/bin/ffmpeg"
PKG = "/home/kw/codePrj/axi_xk264/xk264/scripts/package_nalu.py"
W, H = 640, 480
QP = 28

BAR_Y = [180, 162, 131, 112, 84, 65, 35, 16]
BAR_U = [128, 44, 160, 72, 184, 100, 212, 128]
BAR_V = [128, 142, 16, 58, 198, 212, 114, 128]

def deroll(data, group=8):
    out = bytearray()
    for i in range(0, len(data), group):
        out += data[i:i+group][::-1]
    return bytes(out)

def bar_idx(x, width):
    return ((x % width) * 8) // width

def expected_frame(n):
    y = bytes(BAR_Y[bar_idx(x + 16*n, W)] for yy in range(H) for x in range(W))
    u = bytes(BAR_U[bar_idx(x + 8*n, W//2)] for yy in range(H//2) for x in range(W//2))
    v = bytes(BAR_V[bar_idx(x + 8*n, W//2)] for yy in range(H//2) for x in range(W//2))
    return y, u, v

def psnr(a, b):
    se = sum((x-x2)*(x-x2) for x, x2 in zip(a, b))
    if se == 0:
        return float("inf")
    return 10*math.log10(255*255*len(a)/se)

def main():
    srcdir = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "sim", "runs", "xk640")
    frames = []
    for n in range(2):
        p = os.path.join(srcdir, "bs_frame%d.bin" % n)
        if not os.path.exists(p):
            print("missing", p); return 1
        raw = open(p, "rb").read()
        frames.append(deroll(raw))
        open(os.path.join(srcdir, "core_f%d.bin" % n), "wb").write(frames[-1])
        print("frame %d: %d bytes raw -> %d de-reversed" % (n, len(raw), len(frames[-1])))

    out = os.path.join(srcdir, "bars.h264")
    subprocess.run([sys.executable, PKG, "--w-mbs", str(W//16), "--h-mbs", str(H//16),
                    "--qp", str(QP),
                    "--bs", os.path.join(srcdir, "core_f0.bin"),
                    "--bs", os.path.join(srcdir, "core_f1.bin"),
                    "--out", out], check=True)

    dec = os.path.join(srcdir, "dec.yuv")
    r = subprocess.run([FFMPEG, "-loglevel", "warning", "-i", out,
                        "-pix_fmt", "yuv420p", "-f", "rawvideo", "-y", dec],
                       capture_output=True, text=True)
    if r.stderr.strip():
        print("ffmpeg stderr:")
        print(r.stderr.strip()[:2000])
    data = open(dec, "rb").read()
    fsz = W*H*3//2
    print("decoded %d bytes = %d frames" % (len(data), len(data)//fsz))
    if len(data) < fsz:
        print("FAIL: no frame decoded"); return 1

    for n in range(len(data)//fsz):
        f = data[n*fsz:(n+1)*fsz]
        y = f[:W*H]; u = f[W*H:W*H*5//4]; v = f[W*H*5//4:]
        ey, eu, ev = expected_frame(n)
        print("frame %d PSNR: Y=%.2f U=%.2f V=%.2f" %
              (n, psnr(y, ey), psnr(u, eu), psnr(v, ev)))
    return 0

if __name__ == "__main__":
    sys.exit(main())
