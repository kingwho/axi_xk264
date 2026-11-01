# xk264 V4L2 M2M 驱动调试记录

> 平台：Zynq-7100 / PetaLinux 2020.2 / kernel 5.4.0-xilinx-v2020.2
> 驱动：`software/driver/xk264-v4l2.c`（V4L2 mem2mem 编码器）
> 应用：`software/app/xk264-v4l2-app.c`

本文档记录 xk264 V4L2 M2M 驱动从首次加载到编码成功过程中遇到的三个
关键问题，包含现象、根因、修复方法及涉及的内核 API 细节，供后续维护参考。

---

## 问题一：`open()` 返回 `Invalid argument`

### 现象

应用执行 `open("/dev/videoN", O_RDWR)` 失败，`errno = EINVAL`。
`dmesg` 中无驱动 `open` 相关日志，说明失败发生在 V4L2 核心的队列初始化阶段，
尚未进入驱动的 `xk264_open()`。

### 根因

`struct vb2_ops xk264_qops` 缺少 `.buf_queue` 回调。

kernel 5.4 的 `videobuf2-core.c` 中 `vb2_core_queue_init()` 有强制检查：

```c
// drivers/media/v4l2-core/videobuf2-core.c
int vb2_core_queue_init(struct vb2_queue *q)
{
    ...
    WARN_ON(!q->ops->buf_queue);
    if (WARN_ON(!q->ops->buf_queue))
        return -EINVAL;
    ...
}
```

`buf_queue` 为 NULL 时直接返回 `-EINVAL`，导致 `v4l2_m2m_ctx_init()` →
`xk264_queue_init()` → `vb2_queue_init()` 失败，`open()` 随之失败。

> 说明：在较新的内核（5.10+）中 `buf_queue` 已改为可选，但 5.4 必须提供。

### 修复

在 `xk264_qops` 中添加 `.buf_queue` 回调。

---

## 问题二：编译错误 `-Werror=incompatible-pointer-types`

### 现象

在 `xk264_qops` 中直接赋值 `.buf_queue = v4l2_m2m_buf_queue` 后，
编译报错：

```
error: initialization from incompatible pointer type
        [-Werror=incompatible-pointer-types]
```

### 根因

Xilinx kernel 5.4 中 `v4l2_m2m_buf_queue()` 的函数签名与上游不同：

| 位置 | 签名 |
|------|------|
| `vb2_ops.buf_queue` 期望 | `void (*)(struct vb2_buffer *vb)` |
| 本内核 `v4l2_m2m_buf_queue` 实际 | `void (*)(struct v4l2_m2m_ctx *m2m_ctx, struct vb2_v4l2_buffer *vbuf)` |

两者参数个数和类型均不兼容，无法直接赋值。

> 上游 kernel 5.4 的 `v4l2_m2m_buf_queue` 只接受 `struct vb2_v4l2_buffer *`
> 一个参数（m2m_ctx 通过 `vbuf->vb2_buf.vb2_queue->drv_priv` 隐式获取），
> 但 Xilinx 分支改为显式传入 `m2m_ctx`。

### 修复

编写适配器函数，将单参数的 `vb2_buffer *` 转换为 m2m 核心期望的双参数调用：

```c
static void xk264_buf_queue(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct xk264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

    v4l2_m2m_buf_queue(ctx->m2m_ctx, vbuf);
}

static const struct vb2_ops xk264_qops = {
    .queue_setup      = xk264_queue_setup,
    .buf_prepare      = xk264_buf_prepare,
    .buf_queue        = xk264_buf_queue,   /* ← 适配器 */
    .start_streaming  = xk264_start_streaming,
    .stop_streaming   = xk264_stop_streaming,
};
```

其中 `ctx` 来自 `vb2_queue.drv_priv`，该字段在 `xk264_queue_init()` 中
通过 `src_vq->drv_priv = ctx;` / `dst_vq->drv_priv = ctx;` 设置。

---

## 问题三：`S_FMT OUTPUT` 返回 `Invalid argument`

### 现象

应用设置 OUTPUT 格式（`V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE`，
`V4L2_PIX_FMT_YUV420M`）时：

```
xk264-v4l2-app -d /dev/video3 -W 576 -H 1024 ...
S_FMT OUTPUT: Invalid argument
```

`dmesg` 中只有 `open` 日志，**没有**驱动 `xk264_s_fmt_out()` 的日志，
说明 V4L2 核心在调度到驱动之前就拒绝了请求。

### 根因

驱动 `probe` 中没有设置 `vdev.vfl_dir`，该字段默认为 `VFL_DIR_RX`（0）。

V4L2 核心 `v4l2-ioctl.c` 的 `check_fmt()` 对每种 buffer type 做能力校验：

```c
// drivers/media/v4l2-core/v4l2-ioctl.c
static int check_fmt(struct file *file, enum v4l2_buf_type type)
{
    bool is_rx = vfd->vfl_dir != VFL_DIR_TX;
    bool is_tx = vfd->vfl_dir != VFL_DIR_RX;   /* vfl_dir=RX → is_tx=false */

    switch (type) {
    case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
        if (is_vid && is_tx && ops->vidioc_g_fmt_vid_out_mplane)
            return 0;
        break;
    ...
    }
    return -EINVAL;   /* ← is_tx=false，从这里返回 */
}
```

当 `vfl_dir == VFL_DIR_RX` 时，`is_tx = false`，`check_fmt()` 对所有
OUTPUT 类格式返回 `-EINVAL`，驱动的 `vidioc_s_fmt_vid_out_mplane`
（即 `xk264_s_fmt_out`）永远不会被调用。

同理，`v4l2-dev.c` 的 `determine_valid_ioctls()` 也依赖 `vfl_dir` 来
决定 G_FMT / S_FMT / TRY_FMT 等 OUTPUT 类 ioctl 是否标记为有效。

> M2M 设备必须同时支持 RX（CAPTURE）和 TX（OUTPUT），因此 `vfl_dir`
> 必须设为 `VFL_DIR_M2M`，使 `is_rx` 和 `is_tx` 同时为 true。
> `v4l2_m2m_init()` 和 `video_register_device()` 都不会自动设置该字段，
> 必须由驱动显式赋值。

### 修复

在 `xk264_probe()` 中、`video_register_device()` 之前设置：

```c
x->vdev.vfl_dir = VFL_DIR_M2M;
```

完整上下文：

```c
strscpy(x->vdev.name, "xk264-encoder", sizeof(x->vdev.name));
x->vdev.v4l2_dev     = &x->v4l2_dev;
x->vdev.fops         = &xk264_fops;
x->vdev.ioctl_ops    = &xk264_ioctl_ops;
x->vdev.release      = video_device_release_empty;
x->vdev.lock         = &x->lock;
x->vdev.vfl_dir      = VFL_DIR_M2M;                              /* ← 关键 */
x->vdev.device_caps  = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
video_set_drvdata(&x->vdev, x);

ret = video_register_device(&x->vdev, XK264_VFL_TYPE_VIDEO, -1);
```

设置后 `vfl_dir == VFL_DIR_M2M`（2），`is_rx = is_tx = true`，
CAPTURE 和 OUTPUT 双向的格式 ioctl 均能通过 `check_fmt` 校验。

---

## 总结

| # | 现象 | 根因 | 修复 |
|---|------|------|------|
| 1 | `open()` EINVAL | `vb2_ops.buf_queue` 为 NULL，`vb2_core_queue_init` 强制检查失败 | 添加 `.buf_queue` 回调 |
| 2 | 编译 `incompatible-pointer-types` | Xilinx 内核 `v4l2_m2m_buf_queue` 为 2 参数签名，与 `vb2_ops.buf_queue` 的 1 参数签名不兼容 | 编写 `xk264_buf_queue` 适配器，经 `drv_priv` 解析 `m2m_ctx` 后转发 |
| 3 | `S_FMT OUTPUT` EINVAL | 未设置 `vdev.vfl_dir`，默认 `VFL_DIR_RX` 导致 `check_fmt` 中 `is_tx=false` | `probe` 中设置 `vdev.vfl_dir = VFL_DIR_M2M` |

三个问题均属于 V4L2 / videobuf2 / mem2mem 框架在 kernel 5.4（Xilinx 分支）
下的 API 契约问题，不涉及硬件或编码逻辑本身。修复后驱动可正常完成
`open → S_FMT → S_PARM → S_EXT_CTRLS → REQBUFS → STREAMON → 编码循环`
全流程。
