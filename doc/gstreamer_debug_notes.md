# xk264 GStreamer 接入调试笔记

本文记录 xk264 V4L2 M2M 编码器驱动接入 GStreamer `v4l2h264enc` 元素过程中遇到的问题及修复方案。

## 环境

- 平台：ALIENTEK-ZYNQ（Zynq-7100，PetaLinux 2020.2，kernel 5.4.0）
- 驱动：`xk264-v4l2.ko`，设备节点 `/dev/video3`
- GStreamer：1.14.x / 1.16.x（PetaLinux 默认版本）
- 编码管道：`videotestsrc → v4l2h264enc → h264parse → filesink`

---

## 问题 1：`no element "videotestsrc"`

### 现象

```
$ gst-launch-1.0 videotestsrc num-buffers=60 ! ...
WARNING: erroneous pipeline: no element "videotestsrc"
```

### 排查

```bash
gst-inspect-1.0 | grep -iE "videotestsrc|v4l2h264enc|filesrc|h264parse|videoconvert"
```

输出仅有 `filesrc`（coreelements），说明 rootfs 只安装了 GStreamer 核心，`gst-plugins-base`、`gst-plugins-good`、`gst-plugins-bad` 均未安装。

### 解决方案

在 PetaLinux 工程中启用插件包：

```bash
petalinux-config -c rootfs
# Filesystem Packages → misc → gstreamer
#   勾选 gstreamer1.0-plugins-base  → videotestsrc, videoconvert, videoparse
#   勾选 gstreamer1.0-plugins-good  → v4l2h264enc, h264parse, v4l2src
#   勾选 gstreamer1.0-plugins-bad   → h264parse（部分版本在此）
petalinux-build -c rootfs
```

重新打包烧录 rootfs，或生成 `.ipk` 用 `opkg install` 安装。

| 插件包 | 关键元素 |
|--------|---------|
| `gstreamer1.0-plugins-base` | `videotestsrc`, `videoconvert`, `videoparse`, `audiotestsrc` |
| `gstreamer1.0-plugins-good` | `v4l2h264enc`, `v4l2src`, `h264parse`, `jpegenc` |
| `gstreamer1.0-plugins-bad`  | `h264parse`（部分版本）, `v4l2h264enc`（部分版本） |

---

## 问题 2：`streaming stopped, reason not-negotiated (-4)`

### 现象

```
$ gst-launch-1.0 videotestsrc num-buffers=60 ! \
    video/x-raw,width=640,height=480,format=I420,framerate=30/1 ! \
    v4l2h264enc device=/dev/video3 ! h264parse ! filesink location=out.h264
ERROR: from element /GstPipeline:pipeline0/GstVideoTestSrc:videotestsrc0:
    Internal data stream error.
streaming stopped, reason not-negotiated (-4)
```

`videotestsrc` 与 `v4l2h264enc` 之间 caps 协商失败。

### 排查

```bash
GST_DEBUG=v4l2*:5 gst-launch-1.0 ... 2>&1 | grep "probed caps"
```

观察到 `v4l2h264enc` 的 sink caps 为：

```
probed caps: video/x-raw, format=(string)I420, ... framerate=(fraction)240/1
```

帧率被解析成**固定值 `240/1`**，而管道请求 `framerate=30/1`，固定值无法匹配。

进一步开启详细 debug，发现 GStreamer 调用 `VIDIOC_ENUM_FRAMESIZES` 成功，但帧率来自 `VIDIOC_ENUM_FRAMEINTERVALS`。

### 根因

驱动未实现 `vidioc_enum_framesizes` 和 `vidioc_enum_frameintervals`，GStreamer 无法构建完整的 sink caps 模板。

后续添加 `enum_frameintervals` 后，因 `step` 设置过大导致范围坍缩：

```c
// 错误：step = 1/1 秒
fival->stepwise.min = {1, 240};   // 1/240 s → 240 fps
fival->stepwise.max = {1, 1};     // 1/1 s   → 1 fps
fival->stepwise.step = {1, 1};    // 1/1 s   ← 步长过大
```

从 `1/240` 加一个步长 `1/1` 得到 `241/240 > 1/1`，整个范围内只有 `1/240` 一个有效值，GStreamer 将其坍缩为离散值 `240/1`。

### 解决方案

在驱动中实现 `vidioc_enum_framesizes` 和 `vidioc_enum_frameintervals`，并将 `step` 设为足够小的值：

```c
static int xk264_enum_framesizes(struct file *file, void *priv,
                                 struct v4l2_frmsizeenum *fsize)
{
    if (fsize->index)
        return -EINVAL;
    if (fsize->pixel_format != V4L2_PIX_FMT_YUV420M &&
        fsize->pixel_format != V4L2_PIX_FMT_H264)
        return -EINVAL;

    fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
    fsize->stepwise.min_width  = XK264_MIN_WIDTH;   // 32
    fsize->stepwise.max_width  = 1920;
    fsize->stepwise.step_width = 2;
    fsize->stepwise.min_height = XK264_MIN_HEIGHT;  // 32
    fsize->stepwise.max_height = 1088;
    fsize->stepwise.step_height = 2;
    return 0;
}

static int xk264_enum_frameintervals(struct file *file, void *priv,
                                     struct v4l2_frmivalenum *fival)
{
    if (fival->index)
        return -EINVAL;

    fival->type = V4L2_FRMIVAL_TYPE_STEPWISE;
    fival->stepwise.min.numerator   = 1;
    fival->stepwise.min.denominator = 240;   // 1/240 s → 240 fps
    fival->stepwise.max.numerator   = 1;
    fival->stepwise.max.denominator = 1;     // 1/1 s   → 1 fps
    fival->stepwise.step.numerator   = 1;
    fival->stepwise.step.denominator = 240;  // 步长 1/240 s，覆盖所有整数帧率
    return 0;
}
```

注册到 `v4l2_ioctl_ops`：

```c
.vidioc_enum_framesizes     = xk264_enum_framesizes,
.vidioc_enum_frameintervals = xk264_enum_frameintervals,
```

### 验证

修复后 probed caps 帧率不再是单一固定值。注意 GStreamer 会把 stepwise
**展开成离散值列表**而非连续范围：

```
probed caps: ... framerate=(fraction){ 240/1, 120/1, ..., 30/1, ..., 1/1 }
```

即 `{ 240/k | k=1..240 }`，请求的帧率（如 30/1）在列表中即可协商成功。

---

## 问题 3：caps 看似正常却仍 `not-negotiated`（缺 H264_PROFILE 控件）

### 现象

帧率问题修复后，probed caps 中 `I420 / 640x480 / 30/1` 均可见，管道依旧：

```
ERROR: from element ...GstVideoTestSrc:videotestsrc0: Internal data stream error.
streaming stopped, reason not-negotiated (-4)
```

且 dmesg 中**没有**驱动 `s_fmt_out(in)` 日志——失败发生在 S_FMT 之前。

### 排查

```bash
GST_DEBUG=v4l2:6,v4l2videoenc:6 gst-launch-1.0 videotestsrc num-buffers=10 ! \
  video/x-raw,width=640,height=480,format=I420,framerate=30/1 ! \
  v4l2h264enc ! fakesink 2>&1 | grep -v "probed caps" | tail -40
```

关键日志：

```
DEBUG v4l2videoenc gstv4l2videoenc.c:522:gst_v4l2_video_enc_negotiate: Negotiating H264 profile and level.
WARN  v4l2videoenc gstv4l2videoenc.c:586:gst_v4l2_video_enc_negotiate: Failed to get H264 profile and level: 'Invalid argument'
```

### 根因

`v4l2h264enc` 在 `set_format` 之前先执行 `gst_v4l2_video_enc_negotiate()`，
用 `VIDIOC_G_CTRL` 读取 capture 侧两个控件：

- `V4L2_CID_MPEG_VIDEO_H264_PROFILE`
- `V4L2_CID_MPEG_VIDEO_H264_LEVEL`

驱动只注册了 `GOP_SIZE` / `H264_MIN_QP` / `H264_LEVEL`，缺 `H264_PROFILE`，
V4L2 核心对未注册 CID 的 G_CTRL 返回 `-EINVAL`。

该失败路径在 gstv4l2videoenc 中**只打 WARN、不上报 element error**，caps 事件
被拒后由 basesrc 报出 not-negotiated，表面看像 caps 交集为空，极具迷惑性。

### 解决方案

驱动注册 PROFILE menu 控件。硬件 SPS 由驱动写死 `profile_idc=66`（baseline，
constraint flags=0），菜单只暴露 BASELINE，保持 caps 与码流一致：

```c
#define XK264_DEFAULT_PROFILE	V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE

/* xk264_open()：handler 容量 3 → 4 */
ctx->ctrl_profile = v4l2_ctrl_new_std_menu(&ctx->hdl, &xk264_ctrl_ops,
		V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE, 0,
		XK264_DEFAULT_PROFILE);
```

`ctrl_ops` 仍为空即可——协商只需 G_CTRL，由 V4L2 核心直接返回存储值。

### 验证

`Failed to get H264 profile and level` 警告消失，协商通过，`out.h264` 正常生成，
src caps 中 `profile=(string)baseline` 与 SPS 一致。

### 调试要点

`v4l2h264enc` 协商失败不发 element error 时，直接开
`GST_DEBUG=v4l2:6,v4l2videoenc:6` 观察 `set_format` / `negotiate` 走到哪一步，
并对照 dmesg 判断驱动 S_FMT 是否被调用，可快速区分 caps 层失败与驱动层失败。

---

## 完整可用管道

### 用 videotestsrc 生成彩条编码

```bash
gst-launch-1.0 videotestsrc num-buffers=60 ! \
  video/x-raw,width=640,height=480,format=I420,framerate=30/1 ! \
  v4l2h264enc ! h264parse ! filesink location=out.h264
```

### 用原始 YUV 文件编码（替代 videotestsrc）

若 `videotestsrc` 不可用，可用 `filesrc + videoparse`：

```bash
gst-launch-1.0 \
  filesrc location=input_640x480_I420.yuv ! \
  videoparse format=i420 width=640 height=480 framerate=30/1 ! \
  v4l2h264enc ! h264parse ! filesink location=out.h264
```

### 播放验证

```bash
# 用 ffplay 或 gst-launch 播放生成的 H264 文件
gst-launch-1.0 filesrc location=out.h264 ! h264parse ! avdec_h264 ! autovideosink
```

---

## 常用调试命令

```bash
# 列出所有可用的 GStreamer 元素
gst-inspect-1.0

# 检查 v4l2h264enc 元素属性
gst-inspect-1.0 v4l2h264enc | grep -A5 "device"

# 查看 V4L2 设备支持的格式和帧尺寸
v4l2-ctl -d /dev/video3 --list-formats-ext

# 确认设备节点
cat /sys/class/video4linux/video3/name

# 详细 V4L2 协商日志
GST_DEBUG=v4l2*:5 gst-launch-1.0 ... 2>&1 | grep -iE "caps|negotiat|format"

# 仅过滤 probed caps
GST_DEBUG=v4l2*:5 gst-launch-1.0 ... 2>&1 | grep "probed caps"
```

---

## 注意事项

1. **device 属性只读**：本 GStreamer 版本中 `v4l2h264enc` 的 `device` 属性为只读，gst-launch 传 `device=/dev/video3` 仅触发 `property 'device' ... is not writable` 警告并被忽略。元素会自动枚举并打开支持 H.264 编码的节点（本板即 `/dev/video3`），管道中不要写 `device=`；用 `GST_DEBUG=v4l2*:5 ... 2>&1 | grep -i open` 可确认实际打开的节点。
2. **像素格式**：`videotestsrc` 输出 `I420`（GStreamer 单平面），驱动要求 `YUV420M`（V4L2 多平面），`v4l2h264enc` 内部自动处理转换；若协商失败可加 `videoconvert`。
3. **帧率步长**：`enum_frameintervals` 的 `step` 必须 ≤ `(max - min)`，否则范围坍缩为单个离散值导致协商失败。且 stepwise 会被 GStreamer 展开为 `{ 240/k }` 离散列表——仅 240 的约数帧率（24/30/60…）为整数值，**25fps 不可用**（1/25s 非 1/240s 整数倍）；如需 25fps，step 改为 1/1200s 这类可同时整除 24/25/30/60 的值。
4. **分辨率约束**：驱动 `try_fmt_out` 会根据 MB 数限制（≤ 8040）自动裁剪高度，GStreamer 在 `S_FMT` 后会重新查询格式，通常可正常处理。
