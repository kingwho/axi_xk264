#!/usr/bin/env python3
"""End-to-end bitstream verification for the small (64x64) xk264 AXI sim.

Takes raw DDR dumps bs_frameN.bin (linear byte order, each 8-byte group
byte-reversed because the wrapper assembles 64-bit words big-endian),
de-reverses them, packages into Annex-B NALUs, decodes with ffmpeg and
compares the decoded frames against the testbench's source image.
"""
import subprocess, sys, math, os

FFMPEG = "/home/kw/codePrj/axi_xk264/tools/ffmpeg-master-latest-linux64-gpl/bin/ffmpeg"
PKG = "/home/kw/codePrj/axi_xk264/xk264/scripts/package_nalu.py"
W, H = 64, 64
QP = 28

def deroll(data, group=8):
    out = bytearray()
    for i in range(0, len(data), group):
        out += data[i:i+group][::-1]
    return bytes(out)

def luma_val(fx, fy, frame):
    return (fx*3 + fy*5 + frame*23 + ((fx*fy) & 15)) & 0xFF

def chr_val(fx, fy, frame, uv):
    return (fx*7 + fy*11 + frame*31 + uv*51) & 0xFF

def expected_frame(frame, dx, dy):
    y = bytes(luma_val(x+dx, y+dy, frame) for y in range(H) for x in range(W))
    u = bytes(chr_val(x+dx//2, y+dy//2, frame, 0) for y in range(H//2) for x in range(W//2))
    v = bytes(chr_val(x+dx//2, y+dy//2, frame, 1) for y in range(H//2) for x in range(W//2))
    return y, u, v

def psnr(a, b):
    se = sum((x-x2)*(x-x2) for x, x2 in zip(a, b))
    if se == 0:
        return float("inf")
    return 10*math.log10(255*255*len(a)/se)

def main():
    srcdir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/xk264_small3"
    frames = []
    for n in range(2):
        p = os.path.join(srcdir, "bs_frame%d.bin" % n)
        if not os.path.exists(p):
            print("missing", p); return 1
        raw = open(p, "rb").read()
        frames.append(deroll(raw))
        open(os.path.join(srcdir, "core_f%d.bin" % n), "wb").write(frames[-1])
        print("frame %d: %d bytes raw -> %d de-reversed" % (n, len(raw), len(frames[-1])))

    out = os.path.join(srcdir, "small.h264")
    subprocess.run([sys.executable, PKG, "--w-mbs", "4", "--h-mbs", "4",
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
        if n == 0:
            ey, eu, ev = expected_frame(0, 0, 0)
        else:
            ey, eu, ev = expected_frame(1, 2, 1)
        print("frame %d PSNR: Y=%.2f U=%.2f V=%.2f" %
              (n, psnr(y, ey), psnr(u, eu), psnr(v, ev)))
    return 0

if __name__ == "__main__":
    sys.exit(main())
