#!/usr/bin/env bash
#--------------------------------------------------------------------
# run_sim.sh - xk264 一键仿真脚本（编译 + 运行 + 可选离线打包/验证）
#
# 用法:
#   scripts/run_sim.sh                 # regress: 64x64 I+P 全流程回归
#   scripts/run_sim.sh bars  -n 4 -g 3 # 640x480 滚动彩条, 4 帧 I-P-P-I
#   scripts/run_sim.sh yuv in.yuv -n 2 -q 32
#   scripts/run_sim.sh yuv in.yuv --pkg          # 仿真后打包成可播放 .h264
#   scripts/run_sim.sh bars  --verify            # 解码 PSNR 验证(前 2 帧)
#   scripts/run_sim.sh hostile         # AXI-Lite araddr 竞争回归
#
# 模式:
#   regress   tb_xk264_axi.v      64x64 I+P 帧全流程(DDR 延迟模型)
#   bars      tb_xk264_axi_640.v  640x480 滚动彩条测试源
#   yuv FILE  tb_xk264_axi_640.v  自定义输入: 原始 I420 裸流, 640x480,
#             多帧顺序拼接(与 xk264_app -i 相同格式)
#   hostile   tb_axil_hostile.v   AXI-Lite 读地址竞争(协议回归)
#
# 选项:
#   -n N      帧数(默认 2, 仅 bars/yuv)
#   -g GOP    IDR 间隔(默认 0 = 仅首帧 intra, 仅 bars/yuv)
#   -q QP     量化参数 0..51(默认 28, 仅 bars/yuv)
#   --pkg     仿真后用 xk264_app --sim-dir 把 bs_frame<N>.bin 离线打包
#             成可直接播放的 out.h264(仅 bars/yuv)
#   --verify  bars 模式下解码并用 PSNR 对比彩条期望值(前 2 帧)
#   --nobuild 跳过编译, 直接用 sim/build 下已有的 vvp
#   -- 之后    原样传给 vvp 的额外 plusarg(如 +vcd)
#
# 运行目录 sim/runs/<模式>(每次清空), 日志 sim.log。
# 注意: bars/yuv 两帧约 10~15 分钟(逐字节 DDR 模型较慢)。
#--------------------------------------------------------------------
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD=$ROOT/sim/build
RUNS=$ROOT/sim/runs

MODE=regress
NFRM=2
GOP=0
QP=28
YUV=""
PKG=0
VERIFY=0
NOBUILD=0
EXTRA=()

while [ $# -gt 0 ]; do
    case "$1" in
        regress|bars|hostile) MODE=$1 ;;
        yuv)                  MODE=yuv; shift; YUV=$1 ;;
        -n) shift; NFRM=$1 ;;
        -g) shift; GOP=$1 ;;
        -q) shift; QP=$1 ;;
        --pkg)     PKG=1 ;;
        --verify)  VERIFY=1 ;;
        --nobuild) NOBUILD=1 ;;
        --) shift; EXTRA=("$@"); break ;;
        *) echo "run_sim.sh: unknown option/mode '$1' (see header for usage)" >&2; exit 2 ;;
    esac
    shift
done

case "$MODE" in
    regress|hostile|bars|yuv) ;;
    *) echo "run_sim.sh: bad mode '$MODE'" >&2; exit 2 ;;
esac

# ---------------- TB selection & yuv input validation (fail fast) ----
case "$MODE" in
    regress) SRCS_TB=$ROOT/sim/axi_testbench/tb_xk264_axi.v;    VVPNAME=tb_regress ;;
    hostile) SRCS_TB=$ROOT/sim/axi_testbench/tb_axil_hostile.v; VVPNAME=tb_hostile ;;
    bars)    SRCS_TB=$ROOT/sim/axi_testbench/tb_xk264_axi_640.v; VVPNAME=tb_640 ;;
    yuv)     SRCS_TB=$ROOT/sim/axi_testbench/tb_xk264_axi_640.v; VVPNAME=tb_640 ;;
esac

YUV_AVAIL=0
if [ "$MODE" = yuv ]; then
    YUV=$(readlink -f "$YUV" 2>/dev/null || echo "$YUV")
    if [ ! -f "$YUV" ]; then
        echo "run_sim.sh: yuv input '$YUV' not found" >&2; exit 2
    fi
    SZ=$(stat -c%s "$YUV")
    FRAME_SZ=$((640*480*3/2))
    if [ $((SZ % FRAME_SZ)) -ne 0 ]; then
        echo "run_sim.sh: $YUV size $SZ not a multiple of $FRAME_SZ (I420 640x480)" >&2
        exit 2
    fi
    YUV_AVAIL=$((SZ / FRAME_SZ))
    if [ "$NFRM" -gt "$YUV_AVAIL" ]; then
        echo "run_sim.sh: note: only $YUV_AVAIL frame(s) in file, TB stops at EOF"
    fi
fi

# ---------------- compile ----------------
VVP=$BUILD/$VVPNAME.vvp

need_build=1
if [ "$NOBUILD" = 0 ] && [ -f "$VVP" ]; then
    if ! find "$ROOT/hdl" "$ROOT/lib" "$ROOT/sim/axi_testbench" -name '*.v' \
            -newer "$VVP" | grep -q .; then
        need_build=0
        echo "run_sim: $VVP up to date, skip compile"
    fi
fi

if [ "$NOBUILD" = 1 ]; then
    [ -f "$VVP" ] || { echo "run_sim: $VVP missing (--nobuild)" >&2; exit 2; }
elif [ "$need_build" = 1 ]; then
    mkdir -p "$BUILD"
    echo "run_sim: compiling $MODE TB (~1-2 min)..."
    iverilog -g2005 -I "$ROOT/hdl/core" -o "$VVP" \
        "$SRCS_TB" \
        "$ROOT/hdl/wrapper/xk264_axi_top.v" \
        "$ROOT/hdl/wrapper/xk264_sync_fifo.v" \
        "$ROOT/hdl/wrapper/xk264_axi_rd_arb.v" \
        "$ROOT/hdl/wrapper/xk264_axi_wr_arb.v" \
        "$ROOT/hdl/wrapper/xk264_ext_ctrl.v" \
        $(find "$ROOT/hdl/core" -name '*.v' -print) \
        "$ROOT"/lib/behave/mem/*.v
fi

# ---------------- run ----------------
RUN=$RUNS/$MODE
mkdir -p "$RUN"
rm -f "$RUN"/bs_frame*.bin "$RUN"/*.dat "$RUN"/sim.log "$RUN"/out.h264

PLUSARGS=()
case "$MODE" in
    bars) PLUSARGS=(+nfrm=$NFRM +gop=$GOP +qp=$QP) ;;
    yuv)  PLUSARGS=(+yuv="$YUV" +nfrm=$NFRM +gop=$GOP +qp=$QP) ;;
esac

echo "run_sim: mode=$MODE run=$RUN (log: $RUN/sim.log)"
cd "$RUN"
vvp "$VVP" ${PLUSARGS[@]+"${PLUSARGS[@]}"} ${EXTRA[@]+"${EXTRA[@]}"} 2>&1 | tee sim.log

if ! grep -q "ALL CHECKS PASSED" sim.log; then
    echo "run_sim: *** FAILED (see $RUN/sim.log) ***" >&2
    exit 1
fi
echo "run_sim: ALL CHECKS PASSED"

# ---------------- offline package / verify ----------------
if [ "$PKG" = 1 ]; then
    case "$MODE" in
        bars|yuv)
            make -C "$ROOT/software/app" >/dev/null
            # app 的 -g 语义: intra 每 gop 帧; TB gop=0 表示仅首帧
            APP_GOP=$GOP; [ "$APP_GOP" -le 0 ] && APP_GOP=$NFRM
            "$ROOT/software/app/xk264_app" --sim-dir "$RUN" \
                -W 640 -H 480 -f 30 -q "$QP" -g "$APP_GOP" -o "$RUN/out.h264"
            echo "run_sim: packaged $RUN/out.h264 (ffplay 可直接播放)"
            ;;
        *) echo "run_sim: --pkg only valid for bars/yuv" >&2 ;;
    esac
fi

if [ "$VERIFY" = 1 ]; then
    case "$MODE" in
        bars) python3 "$ROOT/scripts/verify_640.py" "$RUN" ;;
        *)    echo "run_sim: --verify only valid for bars (yuv 无期望参考帧)" >&2 ;;
    esac
fi
