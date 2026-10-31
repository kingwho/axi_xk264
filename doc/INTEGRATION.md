# xk264 AXI 集成说明（Zynq-7100 + PetaLinux 2020.2）

在复旦大学 VIPcore xk264 H.264 编码器核基础上封装 AXI-Lite（控制）+ AXI4
64-bit 主接口（DMA），用于 Zynq-7000（xc7z010/7100 类）Block Design 集成，
并提供 PetaLinux 2020.2（kernel 5.4）平台驱动。

## 目录结构

```
hdl/wrapper/          AXI 封装（本次新增）
  xk264_axi_top.v       顶层：AXI-Lite 寄存器组 + AXI4 读写仲裁 + 帧调度
  xk264_ext_ctrl.v      核内 ext_mem 协议 <-> AXI4 突发转换（DDR 地址映射）
  xk264_axi_rd_arb.v    双客户端读通道仲裁（ext 优先 / 输入预取）
  xk264_axi_wr_arb.v    双客户端写通道仲裁（ext 优先 / 码流写出）
  xk264_sync_fifo.v     FWFT 同步 FIFO
hdl/core/             原核 RTL（cavlc/db/fetch/fme/ime/intra/mc/mem/tq/top）
hdl/core/enc_defines.vh  全核公共宏（Vivado 编译 include 路径须指向 hdl/core/）
lib/synth/mem/        可综合 RAM 模型（ram_1p/ram_2p/rf_2p，替代仿真用
                      lib/behave/mem，接口一致、可推断 BRAM、上电清零）
lib/behave/mem/       仿真用 RAM 模型（已加上电清零）
sim/axi_testbench/    AXI 协议级仿真（含带延迟的 DDR 从机模型）
  tb_xk264_axi.v          64x64 I+P 帧全流程回归
  tb_xk264_axi_640.v      640x480 通用 TB：滚动彩条或 +yuv= 自定义
                          I420 输入（+nfrm/+gop/+qp、bs dump）
  tb_axil_hostile.v       AXI-Lite araddr 竞争回归（IDR 读 0 bug）
  tb_xk264_axi_1080.v     原始 1080 TB（勿直接用，见下文警告）
sim/runs/xk640/       640 TB 一次仿真运行的存档产物（bs dump、解码
                      YUV、PNG、sim.log），可用 scripts/verify_640.py 复验
sim/build/            run_sim.sh 的 vvp 编译产物
scripts/              run_sim.sh（一键仿真，见「仿真」节） /
                      create_bd.tcl（BD 生成）/ package_nalu.py（离线
                      NAL 打包）/ verify_small.py、verify_640.py（解码
                      PSNR 验证，依赖 tools/ 下的 ffmpeg）
doc/INTEGRATION.md    本文档
software/driver/      Linux 内核驱动 + 用户态测试程序（xk264.dtsi 亦在此）
software/driver/recipe/  Yocto/PetaLinux 模块配方
software/app/         应用 demo（分辨率/帧率/GOP 可配，输出可播放码流）
tools/                ffmpeg 静态构建（verify 脚本、码流验证用）
```

仓库根即本目录（原 xk264/ 子目录已扁平化合并，2026-10-31）。

已清理（2026-10-31）：原上游仓库的 `sw/f264.exe`（Windows 参考编码器，
Linux 无用）与 `sim/top_testbench/`（核心级 golden TB，已被 AXI TB 取代）
已删除——git 历史仍可找回；同日完成目录重组（rtl/ → hdl/{core,wrapper}/、
sw/ → software/{app,driver}/、docs/ → doc/、仿真存档收编为 sim/runs/）。

## 寄存器映射（AXI-Lite，32 位）

| 偏移 | 名称     | 说明                                              |
|------|----------|---------------------------------------------------|
| 0x00 | IDR      | 只读，0x02640001                                  |
| 0x04 | CTRL     | [0] start（写 1 启动一帧） [1] irq_en             |
| 0x08 | STATUS   | [0] busy [1] done [2] rd_err [3] wr_err          |
| 0x0C | ISTAT    | 写 1 清零；[0] frame_done（中断源，电平）         |
| 0x10 | CFG      | [5:0] qp [6] intra [7] sys_mode（0=帧模式，1=MB 步进） |
| 0x14 | PIC      | [15:0] mb_x_total [31:16] mb_y_total             |
| 0x18 | YUV      | 输入 YUV 帧物理地址（MB 平铺序）                  |
| 0x1C | BANK0    | 重建帧 bank0 物理地址                             |
| 0x20 | BANK1    | 重建帧 bank1 物理地址                             |
| 0x24 | BSADDR   | 码流输出缓冲物理地址                              |
| 0x28 | BSLEN    | 只读，本帧码流字节数                              |
| 0x2C | FRMCNT   | 只读，已编码帧数                                  |

## DDR 缓冲布局

- 输入 YUV：每宏块 384 字节（32 拍 luma + 16 拍 UV 交错），按 MB 光栅序平铺
- 重建帧 bank：每宏块 512 字节（luma 256 + chroma 256），bank0/bank1 逐帧翻转
  （第 N 帧写 bank[N%2]、参考读 bank[(N-1)%2]）
- 码流：驱动分配，帧结束时 BSLEN 返回有效字节数
- 所有 AXI 突发 ≤16 拍 64-bit INCR，每次一个 outstanding

## DMA 内存与设备树（xk264_dma）

`software/driver/xk264.dtsi` 中保留了一段 DDR 供 DMA 一致性分配：

```dts
reserved-memory {
    #address-cells = <1>;
    #size-cells = <1>;
    ranges;

    xk264_dma: xk264_dma@1f000000 {
        compatible = "shared-dma-pool";
        reg = <0x1f000000 0x0800000>;   /* 0x1F000000, 8 MiB */
        reusable;
    };
};

&amba {
    xk264_enc: xk264@43c00000 {
        ...
        memory-region = <&xk264_dma>;
    };
};
```

各属性含义：

| 属性 | 作用 |
|------|------|
| `reserved-memory` 父节点 | 声明一段内核不可分配的物理内存；`#address-cells/size-cells = <1>` + `ranges` 表示子节点 `reg` 是绝对物理地址（Zynq-7000 为 32 位地址） |
| `reg = <0x1f000000 0x0800000>` | 保留 DDR 中 `0x1F000000 ~ 0x1F7FFFFF` 共 8 MiB（512 MiB DDR 的顶部 8 MiB） |
| `compatible = "shared-dma-pool"` | 内核 reserved-mem 子系统将其设置为设备的专用 DMA 一致性内存池 |
| `reusable` | 池空闲时可归还内核页分配器（CMA 语义），非永久挖掉 |
| `memory-region = <&xk264_dma>` | 把池绑定到 `xk264` 设备；此后该设备的 `dma_alloc_coherent` 只从此池分配 |

驱动侧（`xk264.c` probe）：`dmam_alloc_coherent()` 一次性分配整个区域
（`XK264_REGION_SIZE` = YUV 输入 + 2 个重建 bank + 码流输出，FHD 下约
7 MiB，正好放入 8 MiB 池），然后把返回的 DMA 物理地址 + 各 buffer 偏移
直接写入硬件寄存器 `YUV/BANK0/BANK1/BSADDR`，PL 经 S_AXI_HP0 直接访问
这段 DDR。一致性内存保证硬件与 CPU 可见性同步，无需 cache flush。

### 确认 Zynq-7100 DDR 起止地址

Zynq-7000 的 DDR 物理映射：低端窗口从 **0x00000000** 起，最多 1 GiB
（0x0 ~ 0x3FFFFFFF）；超出 1 GiB 的部分映射到高端窗口
0x40000000 ~ 0x7FFFFFFF。典型 7z010/7z020 板卡为 512 MiB DDR，
即 `0x00000000 ~ 0x1FFFFFFF`——dtsi 里的 `0x1F000000` 即按
"512 MiB DDR 顶部"设定。确认方法（按推荐顺序）：

1. **目标板运行中的系统**（最直接）：
   ```bash
   cat /proc/iomem        # "System RAM" 行，如 00000000-1fffffff : System RAM
   dmesg | grep -i memory # 启动时打印的 DDR 容量
   ```
   正常还能看到 `1f000000-1f7fffff : xk264_dma`（保留区以节点名出现在
   /proc/iomem）。
2. **U-Boot**：`bdinfo` 打印 `bi_dram` 各 bank 的 start/size。
3. **PetaLinux 生成的设备树**：`build/components/plnx_workspace/
   device-tree-generation/` 下 `pcw.dtsi` / `system-top.dts` 的 `memory`
   节点 `reg`（由 Vivado PS7 DDR 配置 / PCW 预设生成）。
4. **Vivado 工程源头**：Block Design 中 PS 的 `DDR controller →
   Memory Part` 决定容量；`ps7_init.tcl` 或 address editor 中 S_AXI_HP0
   的地址范围（HP 口 0x0 起始映射到 DDR 偏移 0）。

注意：若板卡实际为 1 GiB DDR，`0x1F000000` 不再是"顶部"——保留区落在
DDR 中部（功能正常，但失去避开内核常用区的设计意图），应改为
`0x3f000000`。用 `/proc/iomem` 确认后对齐即可。

## 仿真

### 一键脚本（推荐）

`scripts/run_sim.sh` 合并了编译（iverilog）+ 运行 + 可选离线打包/解码
验证，产物与日志统一放在 `sim/runs/<模式>/`：

```bash
scripts/run_sim.sh                        # regress: 64x64 I+P 全流程回归
scripts/run_sim.sh bars -n 4 -g 3         # 640x480 滚动彩条, I-P-P-I
scripts/run_sim.sh bars --verify          # 彩条 + 解码 PSNR 验证(前 2 帧)
scripts/run_sim.sh hostile                # AXI-Lite araddr 竞争回归
# 自定义 YUV 输入: 原始 I420 裸流, 640x480, 多帧拼接(同 xk264_app -i 格式)
scripts/run_sim.sh yuv in.yuv -n 2 -q 32
scripts/run_sim.sh yuv in.yuv --pkg       # 仿真后离线打包 out.h264 可直接播放
scripts/run_sim.sh bars -- +vcd           # '--' 后的参数原样传给 vvp
```

选项：`-n` 帧数（默认 2）、`-g` IDR 间隔（默认 0 = 仅首帧 intra）、
`-q` QP（默认 28，仅 bars/yuv）、`--pkg` 离线打包、`--verify` 彩条
PSNR 验证、`--nobuild` 跳过编译。yuv 模式会先校验文件尺寸是
460800（640*480*3/2）的整数倍再编译。两帧约 10~15 分钟（逐字节 DDR
模型较慢）；进度看 `sim/runs/<模式>/bs_frame<N>.bin` 的出现，不要等
日志（vvp 重定向到文件时是块缓冲）。

### 手动命令（等价参考）

```bash
# 行为级 RAM 模型（已加上电清零，匹配 FPGA BRAM）
iverilog -g2005 -I hdl/core -o /tmp/tb.vvp \
    sim/axi_testbench/tb_xk264_axi.v \
    hdl/wrapper/xk264_axi_top.v hdl/wrapper/xk264_sync_fifo.v \
    hdl/wrapper/xk264_axi_rd_arb.v hdl/wrapper/xk264_axi_wr_arb.v \
    hdl/wrapper/xk264_ext_ctrl.v \
    $(find hdl/core -name '*.v' -print) \
    lib/behave/mem/*.v
vvp /tmp/tb.vvp            # 期望: *** ALL CHECKS PASSED ***
```

测试内容：I 帧 + P 帧全流程（输入 DMA、参考帧读写、去方块、码流写回、
中断、寄存器检查），DDR 从机模型带 12/10 拍读写延迟。
`tb_xk264_axi_640.v` 支持：
- `+nfrm=N +gop=M`（每 M 帧 IDR，验证多 GOP 序列 I-P-P-I）；
- `+qp=N` 量化参数；
- `+yuv=<file>` 用户自定义输入：原始 I420 裸流（640x480，多帧顺序
  拼接，Y/U/V 平面布局与 `xk264_app -i` 完全相同），逐帧 `$fread`
  读入后由 TB 转成 wrapper 的 MB 平铺序（与应用 `planar_to_tiled()`
  同一映射），文件不足 `+nfrm` 帧时到 EOF 提前停止；
- 码流 dump 到 bs_frame<N>.bin 可用 `--pkg`（内部调
  `software/app/xk264_app --sim-dir`）离线打包验证。

注意：仿真必须使用带清零的 RAM 模型（本仓库已修改）。搜索窗 RAM 在
I 帧后首个 P 帧只被部分写入，未写地址读出值必须确定（FPGA BRAM 上电为 0），
否则 X 会传播进 MV 导致 MC 色度 FSM 死锁。

注意：`tb_xk264_axi_1080.v` 仍含 `always @(*)` 读大数组（2M 字 DDR 模型）
的旧写法，隐式敏感列表会膨胀到每个数组字，编译 >1h 不结束——直接用会
挂死。要跑 1080 规模请参照 `tb_xk264_axi_640.v` 的写法（读用连续
`assign`、RMW 写用时钟块内 merge 函数）改写后再用。

## Vivado Block Design

```bash
vivado -mode batch -source scripts/create_bd.tcl
# 生成 xk264_proj/，BD: PS7 GP0->AXI-Lite@0x43C00000,
# xk264 M_AXI->HP0(64bit DDR 512M), irq->IRQ_F2P, FCLK 100MHz
launch_runs synth_1 impl_1 -to_step write_bitstream
```

### 需要加入 Vivado 综合的源文件清单

手工建工程（GUI）时，**Design Sources 需加入且仅需加入以下三组**
（共 134 个 .v + 1 个 .vh）：

| 组 | 内容 | 说明 |
|----|------|------|
| 1 | `hdl/wrapper/`（5 个 .v） | AXI 封装：`xk264_axi_top.v`（顶层，内部例化原核 `top`）、`xk264_ext_ctrl.v`、`xk264_axi_rd_arb.v`、`xk264_axi_wr_arb.v`、`xk264_sync_fifo.v` |
| 2 | `hdl/core/` 10 个子目录（126 个 .v） | 原核 RTL：`top` 6 / `cavlc` 27 / `db` 9 / `fetch` 6 / `fme` 10 / `ime` 14 / `intra` 13 / `mc` 8 / `mem` 15 / `tq` 18 |
| 3 | `lib/synth/mem/`（3 个 .v） | 可综合 RAM 模型 `ram_1p` / `ram_2p` / `rf_2p`，原核 `rtl/mem/` 下的封装模块例化它们 |

头文件 `hdl/core/enc_defines.vh` **不必**作为 Design Source 加入（RTL 以
`` `include "enc_defines.vh" `` 引用，靠 include 搜索路径解析）；随目录
一起添加时 Vivado 会自动识别为 Header 类型，加进工程仅便于编辑跳转。

**绝对不要加入综合的文件**：

- `lib/behave/mem/` —— 仿真用 RAM 模型，与 `lib/synth/mem/` **模块同名**
  （ram_1p/ram_2p/rf_2p），一起加入会报重复定义错误；
- `sim/`（testbench）、`software/`、`scripts/`、`doc/` —— 与综合无关。

**两个必配的编译设置**（Settings → General → Verilog options，缺一个
综合必失败）：

1. **Include search path** 添加 `hdl/core/`（绝对路径）——所有 RTL 裸
   `` `include "enc_defines.vh" ``，Vivado 只搜索源文件所在目录，不配
   路径时首个症状是 `BitStream_packer.v` 报打不开 enc_defines.vh 的
   "语法错误"；
2. **Verilog defines** 填 `RTL_MODEL=1` 与 `BIT_DEPTH=8` —— 选择
   `lib/synth/mem` 的 RTL_MODEL 分支；不配时 RAM 封装走 FPGA_MODEL
   分支，例化本仓库不存在的生成式 RAM IP，报未解析模块。

GUI 操作顺序：Create Project（RTL project，part 按板卡，如
xc7z010clg400-1）→ Add Sources → Add or create design sources →
**Add Directories** 依次选 `hdl/wrapper`、`hdl/core`（递归收入全部 .v，
enc_defines.vh 自动归为 Header）→ 再 Add Files 单独确认
`lib/synth/mem/` 3 个文件 → 配好上述两个 Verilog options →
Reset Runs → Run Synthesis。顶层模块为 `xk264_axi_top`（建 BD 时以
module reference 方式例化，见 create_bd.tcl）。

## PetaLinux 2020.2

1. **设备树**：把 `software/driver/xk264.dtsi` 内容并入
   `project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi`
   （控制基址 0x43C00000，中断 SPI 61，reserved-memory 供 DMA 连续分配，
   详见「DMA 内存与设备树」一节）

2. **内核模块**：
   ```bash
   cp -r software/driver/recipe project-spec/meta-user/recipes-modules/xk264
   cp software/driver/{xk264.c,xk264_regs.h,Makefile} \
      project-spec/meta-user/recipes-modules/xk264/files/
   petalinux-config -c rootfs   # 启用 kernel-module-xk264
   petalinux-build
   ```

3. **用户态**：
   ```bash
   ${CROSS}gcc software/driver/xk264_test.c -o xk264_test   # 或在目标机编译
   ./xk264_test out.h264    # I 帧 + P 帧，打印码流长度
   ```

   **应用 demo（software/app/xk264_app.c）**：可配置分辨率/帧率/GOP/QP，
   输出可直接播放的 Annex-B 码流：
   ```bash
   cd software/app && make CROSS=arm-linux-gnueabihf-
   # 目标机：内置滚动彩条测试源，640x480@30fps，GOP 30
   ./xk264_app -W 640 -H 480 -f 30 -g 30 -q 28 -o out.h264
   # 从原始 I420 文件编码
   ./xk264_app -W 1280 -H 720 -f 25 -g 50 -i input.yuv -o out.h264
   xk264-encode  -W 576 -H 1024 -f 25 -g 50 -i input_576x1024_YUV420P\ \(I420\).yuv -o out.h264
   
   ffplay out.h264
   ```
   应用在用户态补齐 H.264 语法（SPS/PPS/slice header、防竞争字节、
   Annex-B 起始码，SPS 带 VUI 时序信息与 crop），并把 DDR 中按
   64-bit 大端拼字的码流每 8 字节一组去反转后打包；按 -f 设定的帧率
   节奏编码并统计实际帧率与码率。`--sim-dir DIR` 可离线打包仿真
dump 的 bs_frame<N>.bin（主机验证，无需硬件）。非 16 倍数分辨率
   自动边缘复制填充至整宏块并由 SPS crop 还原显示尺寸。
   
   **YUV 输入格式（-i 指定的文件）**：仅接受**原始 I420（YUV420P）
   裸流**——8-bit、4:2:0 采样的平面格式，无任何文件头/长度信息：
   - 每帧依次为整幅 Y 平面、U 平面、V 平面，多帧顺序拼接；
   - 每帧 `W*H*3/2` 字节，U/V 平面各 `W/2 x H/2`（奇数宽高按
     `(W+1)/2` 向上取整）；
   - 画面尺寸完全由 `-W/-H` 命令行参数给出，文件本身不含尺寸，
     二者不匹配时会按帧错位读取；
   - 其余常见格式（NV12 半平面、YUYV/UYVY 打包、图片/视频容器）
     **不被识别**，需先转换：
     ```bash
     ffmpeg -i in.mp4 -f rawvideo -pix_fmt yuv420p input.yuv
     ffmpeg -i in.png -f rawvideo -pix_fmt yuv420p input.yuv
     # NV12 -> I420
     ffmpeg -f rawvideo -pix_fmt nv12 -s 1280x720 -i in.nv12 \
            -f rawvideo -pix_fmt yuv420p input.yuv
     ```
   - 不指定 `-i` 时无需输入文件，应用内置 BT.601 75% 八彩条滚动
     测试源（luma 每帧移 16px、chroma 移 8px），默认编 60 帧（`-n`
     可改）。

驱动接口：`/dev/xk264`（misc 设备）
- `mmap`：偏移 0 = 输入 YUV 区（~3MB @1080p），偏移 0x30D800 = 码流区（1MB），
  大小由 `XK264_IOC_G_INFO` 返回
- `XK264_IOC_S_PARAM`：设置 mb_x/mb_y_total、qp
- `XK264_IOC_ENCODE`：{flags(intra), timeout_ms} 入，{bs_len} 出；
  阻塞等待帧完成中断（ISTAT w1c 应答）

## 已知集成点（对原核的修改）

- `hdl/core/top/top_ctrl.v` I_S2：去掉 intra_done_flag 条件（DDR 延迟下死锁，
  见文件内注释）
- `hdl/wrapper/xk264_axi_top.v` 码流尾部字 `tail_mask`：移位量应为 `nb*8`
  而非 `64-nb*8`。原式在尾字节数 nb=5..7 时把有效字节清零——
  码流长度 %8 为 5..7 的帧（约 3/8 概率）最后一个宏块损坏，
  解码器报 "corrupted macroblock"（2026-10-24 经 I-P-P-I 仿真 +
  ffmpeg 解码发现并修复，nb≤4 时旧式恰好被 wstrb 掩盖）
- `lib/behave/mem/*.v`：RAM 数组上电清零（仿真正确性，对应 BRAM 上电 0）
- `lib/synth/mem/`：新增可综合 RAM 模型（原 FPGA_MODEL 分支依赖未提供的
  生成式 RAM IP，故统一走 RTL_MODEL + 本目录模型）

## 性能参考

仿真（64x64, 4x4 MB, QP28）：I 帧 ~1400 B，P 帧 ~1950 B；
DDR 延迟 12/10 拍下吞吐受 ext_mem 串行化限制，720p@30fps 量级（100MHz）。
