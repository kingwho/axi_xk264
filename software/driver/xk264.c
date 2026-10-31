// SPDX-License-Identifier: GPL-2.0
/*
 * xk264 - platform driver for the Fudan VIPcore xk264 H.264 baseline
 * encoder wrapped with AXI-Lite control + AXI4 master DMA (see
 * hdl/wrapper/xk264_axi_top.v).
 *
 * The IP is a bus master: it fetches the input YUV frame and writes
 * the bitstream / reconstructed frames directly to DDR over its own
 * AXI HP port. The driver therefore allocates DMA-coherent buffers,
 * maps them into user space, programs the address registers and waits
 * for the frame-done interrupt.
 *
 * Buffer layout inside one coherent region (sizes for W x H macroblocks):
 *   XK264_OFF_YUV     MBs * 384 bytes   input frame, MB-tiled order
 *   XK264_OFF_BANK0   MBs * 512 bytes   reconstruction bank 0
 *   XK264_OFF_BANK1   MBs * 512 bytes   reconstruction bank 1
 *   XK264_OFF_BS      XK264_BS_SIZE     bitstream output
 *
 * Target: PetaLinux 2020.2 / kernel 5.4, Zynq-7000 (zynq7100 class).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include "xk264_regs.h"

#define DRV_NAME "xk264"

/*
 * Buffer layout inside the coherent region (bytes).  Every sub-region
 * starts on a page boundary and is a whole number of pages long, so
 * each one can be mmap'ed on its own: mmap() rejects offsets that are
 * not page aligned and compares the (page rounded) length against the
 * region size.
 */
#define XK264_BS_SIZE		SZ_1M
#define XK264_MAX_MBS		(1920u / 16u) * (1080u / 16u) /* 8040 MBs, FHD */
#define XK264_MAX_YUV_SIZE	(XK264_MAX_MBS * 384u)
#define XK264_MAX_BANK_SIZE	(XK264_MAX_MBS * 512u)

#define XK264_REGION_YUV_SIZE	((u32)PAGE_ALIGN(XK264_MAX_YUV_SIZE))
#define XK264_REGION_BANK_SIZE	((u32)PAGE_ALIGN(XK264_MAX_BANK_SIZE))
#define XK264_REGION_BS_SIZE	((u32)PAGE_ALIGN(XK264_BS_SIZE))

#define XK264_OFF_YUV		0x00000000u
#define XK264_OFF_BANK0		(XK264_OFF_YUV         + XK264_REGION_YUV_SIZE)
#define XK264_OFF_BANK1		(XK264_OFF_BANK0       + XK264_REGION_BANK_SIZE)
#define XK264_OFF_BS		(XK264_OFF_BANK1       + XK264_REGION_BANK_SIZE)
#define XK264_REGION_SIZE	(XK264_OFF_BS         + XK264_REGION_BS_SIZE)

/* mmap index -> offset, page aligned */
#define XK264_MMAP_YUV		0
#define XK264_MMAP_BS		1
#define XK264_MMAP_N		2

struct xk264_param {
	__u16 mb_x_total;	/* picture width in macroblocks (>=1) */
	__u16 mb_y_total;	/* picture height in macroblocks (>=1) */
	__u8  qp;		/* 0..51 */
	__u8  reserved[3];
};

struct xk264_encode {
	__u32 flags;		/* XK264_ENC_INTRA */
	__u32 bs_len;		/* out: bitstream bytes written */
	__u32 timeout_ms;
	__u32 reserved;
};

#define XK264_ENC_INTRA		BIT(0)

#define XK264_IOC_MAGIC		'x'
#define XK264_IOC_G_INFO	_IOR(XK264_IOC_MAGIC, 0x00, struct xk264_info)
#define XK264_IOC_S_PARAM	_IOW(XK264_IOC_MAGIC, 0x01, struct xk264_param)
#define XK264_IOC_ENCODE	_IOWR(XK264_IOC_MAGIC, 0x02, struct xk264_encode)

struct xk264_info {
	__u32 hw_id;		/* reads IDR register, expect 0x02640001 */
	__u32 max_mbs;
	__u32 mmap_off[XK264_MMAP_N];
	__u32 mmap_len[XK264_MMAP_N];
};

struct xk264_dev {
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	struct miscdevice	misc;

	void			*coherent;
	dma_addr_t		coherent_dma;
	u32			region_size;

	struct mutex		lock;		/* serializes encode */
	wait_queue_head_t	done_q;
	bool			frame_done;
	u32			frame_error;

	struct xk264_param	param;
};

static inline void xk264_write(struct xk264_dev *x, u32 reg, u32 val)
{
	iowrite32(val, x->base + reg);
}

static inline u32 xk264_read(struct xk264_dev *x, u32 reg)
{
	return ioread32(x->base + reg);
}

/* ------------------------------------------------------------------ */
/* IRQ                                                                */
/* ------------------------------------------------------------------ */

static irqreturn_t xk264_irq(int irq, void *data)
{
	struct xk264_dev *x = data;
	u32 istat = xk264_read(x, XK264_REG_ISTAT);
	irqreturn_t ret = IRQ_NONE;

	dev_dbg(x->dev, "irq %d: istat %#x\n", irq, istat);

	if (istat & XK264_ISTAT_FRAME_DONE) {
		/* w1c: clears the level IRQ */
		xk264_write(x, XK264_REG_ISTAT, XK264_ISTAT_FRAME_DONE);
		x->frame_error = xk264_read(x, XK264_REG_STATUS) &
				 (XK264_STATUS_RD_ERR | XK264_STATUS_WR_ERR);
		x->frame_done = true;
		if (x->frame_error)
			dev_err(x->dev, "frame done with DMA error, status %#x\n",
				xk264_read(x, XK264_REG_STATUS));
		wake_up_interruptible(&x->done_q);
		ret = IRQ_HANDLED;
	}
	return ret;
}

/* ------------------------------------------------------------------ */
/* frame encode                                                       */
/* ------------------------------------------------------------------ */

static int xk264_encode_frame(struct xk264_dev *x, struct xk264_encode *enc)
{
	u32 status;
	/*
	 * cfg[7] = sys_mode: 0 = frame mode, 1 = MB step mode.  In MB step
	 * mode top_ctrl only advances on a high level of sys_start, but the
	 * wrapper pulses sys_start for a single cycle per frame, so the core
	 * would stay in LOAD forever (busy=1, no frame done).  Keep it 0.
	 */
	u32 cfg = (x->param.qp & 0x3f)
		| ((enc->flags & XK264_ENC_INTRA) ? BIT(6) : 0);
	unsigned long timeout =
		msecs_to_jiffies(enc->timeout_ms ? : 10000);
	int ret;

	/* make sure previous frame finished and errors are visible */
	status = xk264_read(x, XK264_REG_STATUS);
	if (status & XK264_STATUS_BUSY) {
		dev_err(x->dev, "encode: still busy, status %#x\n", status);
		return -EBUSY;
	}

	x->frame_done = false;
	x->frame_error = 0;

	xk264_write(x, XK264_REG_ISTAT, XK264_ISTAT_FRAME_DONE); /* w1c */
	xk264_write(x, XK264_REG_CFG, cfg);
	xk264_write(x, XK264_REG_PIC,
		    (x->param.mb_y_total << 16) | x->param.mb_x_total);
	xk264_write(x, XK264_REG_YUV,   x->coherent_dma + XK264_OFF_YUV);
	xk264_write(x, XK264_REG_BANK0, x->coherent_dma + XK264_OFF_BANK0);
	xk264_write(x, XK264_REG_BANK1, x->coherent_dma + XK264_OFF_BANK1);
	xk264_write(x, XK264_REG_BSADDR, x->coherent_dma + XK264_OFF_BS);

	dev_dbg(x->dev,
		"encode: %ux%u MBs, qp %u, flags %#x, cfg %#x, yuv %#x, bank0 %#x, bank1 %#x, bs %#x\n",
		x->param.mb_x_total, x->param.mb_y_total, x->param.qp,
		enc->flags, cfg, (u32)(x->coherent_dma + XK264_OFF_YUV),
		(u32)(x->coherent_dma + XK264_OFF_BANK0),
		(u32)(x->coherent_dma + XK264_OFF_BANK1),
		(u32)(x->coherent_dma + XK264_OFF_BS));

	dma_wmb();

	/* start + irq enable; keep irq_en across the RMW */
	xk264_write(x, XK264_REG_CTRL, XK264_CTRL_START | XK264_CTRL_IRQ_EN);

	ret = wait_event_interruptible_timeout(x->done_q, x->frame_done,
					       timeout);
	if (ret < 0)
		return ret;
	if (ret == 0) {
		dev_err(x->dev,
			"encode timeout after %u ms: status %#x, istat %#x, frmcnt %u\n",
			enc->timeout_ms ? : 10000,
			xk264_read(x, XK264_REG_STATUS),
			xk264_read(x, XK264_REG_ISTAT),
			xk264_read(x, XK264_REG_FRMCNT));
		return -ETIMEDOUT;
	}
	if (x->frame_error) {
		dev_err(x->dev, "encode failed: DMA rd/wr error %#x\n",
			x->frame_error);
		return -EIO;
	}

	enc->bs_len = xk264_read(x, XK264_REG_BSLEN);
	dev_dbg(x->dev, "encode done: bs_len %u\n", enc->bs_len);
	return 0;
}

/* ------------------------------------------------------------------ */
/* file operations                                                    */
/* ------------------------------------------------------------------ */

static int xk264_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct xk264_dev *x = container_of(misc, struct xk264_dev, misc);

	file->private_data = x;
	dev_dbg(x->dev, "open by pid %d\n", current->pid);
	return 0;
}

static long xk264_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct xk264_dev *x = file->private_data;
	void __user *uarg = (void __user *)arg;
	long ret = 0;

	switch (cmd) {
	case XK264_IOC_G_INFO: {
		struct xk264_info info;

		info.hw_id = xk264_read(x, XK264_REG_IDR);
		info.max_mbs = XK264_MAX_MBS;
		info.mmap_off[XK264_MMAP_YUV] = XK264_OFF_YUV;
		info.mmap_off[XK264_MMAP_BS] = XK264_OFF_BS;
		info.mmap_len[XK264_MMAP_YUV] = XK264_REGION_YUV_SIZE;
		info.mmap_len[XK264_MMAP_BS] = XK264_REGION_BS_SIZE;
		if (copy_to_user(uarg, &info, sizeof(info)))
			ret = -EFAULT;
		break;
	}
	case XK264_IOC_S_PARAM: {
		struct xk264_param p;

		if (copy_from_user(&p, uarg, sizeof(p)))
			return -EFAULT;
		if (p.mb_x_total == 0 || p.mb_y_total == 0 ||
		    p.mb_x_total * p.mb_y_total > XK264_MAX_MBS) {
			dev_err(x->dev, "bad param: %ux%u MBs (max %u)\n",
				p.mb_x_total, p.mb_y_total, XK264_MAX_MBS);
			return -EINVAL;
		}
		if (p.qp > 51) {
			dev_err(x->dev, "bad param: qp %u (max 51)\n", p.qp);
			return -EINVAL;
		}
		mutex_lock(&x->lock);
		x->param = p;
		mutex_unlock(&x->lock);
		dev_info(x->dev, "param set: %ux%u MBs, qp %u\n",
			 p.mb_x_total, p.mb_y_total, p.qp);
		break;
	}
	case XK264_IOC_ENCODE: {
		struct xk264_encode enc;

		if (copy_from_user(&enc, uarg, sizeof(enc)))
			return -EFAULT;
		mutex_lock(&x->lock);
		ret = xk264_encode_frame(x, &enc);
		mutex_unlock(&x->lock);
		if (ret == 0 && copy_to_user(uarg, &enc, sizeof(enc)))
			ret = -EFAULT;
		break;
	}
	default:
		dev_dbg(x->dev, "unknown ioctl cmd %#x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}
	if (ret)
		dev_dbg(x->dev, "ioctl cmd %#x -> %ld\n", cmd, ret);
	return ret;
}

static int xk264_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct xk264_dev *x = file->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long region_off;
	size_t region_len;
	int ret;

	if (off == XK264_OFF_YUV) {
		region_off = XK264_OFF_YUV;
		region_len = XK264_REGION_YUV_SIZE;
	} else if (off == XK264_OFF_BS) {
		region_off = XK264_OFF_BS;
		region_len = XK264_REGION_BS_SIZE;
	} else {
		dev_err(x->dev, "mmap: bad offset %#lx\n", off);
		return -EINVAL;
	}
	if (size > region_len) {
		dev_err(x->dev, "mmap: size %#lx exceeds region %zu at %#lx\n",
			size, region_len, off);
		return -EINVAL;
	}

	dev_dbg(x->dev, "mmap: off %#lx size %#lx dma %pad\n",
		off, size, &x->coherent_dma);

	/*
	 * dma_mmap_coherent() treats vma->vm_pgoff as an offset inside the
	 * buffer it is handed.  We already add region_off to the cpu/dma
	 * addresses, so the page offset must be cleared or the kernel
	 * rejects the mapping with -ENXIO (offset >= buffer pages).
	 */
	vma->vm_pgoff = 0;
	ret = dma_mmap_coherent(x->dev, vma,
				x->coherent + region_off,
				x->coherent_dma + region_off,
				size);
	if (ret)
		dev_err(x->dev, "mmap: dma_mmap_coherent failed: %d\n", ret);
	return ret;
}

static const struct file_operations xk264_fops = {
	.owner		= THIS_MODULE,
	.open		= xk264_open,
	.unlocked_ioctl	= xk264_ioctl,
	.mmap		= xk264_mmap,
};

/* ------------------------------------------------------------------ */
/* platform driver                                                    */
/* ------------------------------------------------------------------ */

static int xk264_probe(struct platform_device *pdev)
{
	struct xk264_dev *x;
	struct resource *res;
	u32 idr;
	int ret;

	dev_info(&pdev->dev, "%s: probe start (kernel %s)\n",
		 DRV_NAME, utsname()->release);

	x = devm_kzalloc(&pdev->dev, sizeof(*x), GFP_KERNEL);
	if (!x) {
		dev_err(&pdev->dev, "%s: devm_kzalloc(%zu) failed\n",
			DRV_NAME, sizeof(*x));
		return -ENOMEM;
	}
	dev_dbg(&pdev->dev, "%s: device state at %px\n", DRV_NAME, x);

	x->dev = &pdev->dev;
	mutex_init(&x->lock);
	init_waitqueue_head(&x->done_q);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "%s: no IORESOURCE_MEM resource #0\n",
			DRV_NAME);
		return -ENODEV;
	}
	dev_info(&pdev->dev, "%s: MEM resource %pR\n", DRV_NAME, res);

	x->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(x->base)) {
		dev_err(&pdev->dev, "%s: devm_ioremap_resource(%pR) failed: %ld\n",
			DRV_NAME, res, PTR_ERR(x->base));
		return PTR_ERR(x->base);
	}
	dev_info(&pdev->dev, "%s: AXI-Lite mapped at %p\n", DRV_NAME, x->base);

	/* sanity check the wrapper's ID register */
	idr = ioread32(x->base + XK264_REG_IDR);
	if (idr != XK264_IDR_MAGIC) {
		dev_err(&pdev->dev, "bad ID register %#x (expected %#x)\n",
			idr, XK264_IDR_MAGIC);
		return -ENODEV;
	}
	dev_info(&pdev->dev, "%s: ID register %#x, status %#x, frmcnt %u\n",
		 DRV_NAME, idr, ioread32(x->base + XK264_REG_STATUS),
		 ioread32(x->base + XK264_REG_FRMCNT));

	x->irq = platform_get_irq(pdev, 0);
	if (x->irq < 0) {
		dev_err(&pdev->dev, "%s: platform_get_irq failed: %d\n",
			DRV_NAME, x->irq);
		return x->irq;
	}
	dev_info(&pdev->dev, "%s: irq %d\n", DRV_NAME, x->irq);

	x->region_size = XK264_REGION_SIZE;
	dev_info(&pdev->dev,
		 "%s: DMA region %u bytes (%u KiB): yuv %#x+%u, bank0 %#x+%u, bank1 %#x+%u, bs %#x+%u\n",
		 DRV_NAME, x->region_size, x->region_size >> 10,
		 XK264_OFF_YUV, XK264_REGION_YUV_SIZE,
		 XK264_OFF_BANK0, XK264_REGION_BANK_SIZE,
		 XK264_OFF_BANK1, XK264_REGION_BANK_SIZE,
		 XK264_OFF_BS, XK264_REGION_BS_SIZE);
	dev_info(&pdev->dev, "%s: dma mask %#llx, memory-region property %s\n",
		 DRV_NAME, (unsigned long long)dma_get_mask(&pdev->dev),
		 of_property_read_bool(pdev->dev.of_node, "memory-region") ?
		 "present" : "absent");

	x->coherent = dmam_alloc_coherent(&pdev->dev, x->region_size,
					  &x->coherent_dma, GFP_KERNEL);
	if (!x->coherent) {
		dev_err(&pdev->dev,
			"%s: dmam_alloc_coherent(%u bytes) failed: check CMA size / reserved-memory pool size\n",
			DRV_NAME, x->region_size);
		return -ENOMEM;
	}
	dev_info(&pdev->dev, "%s: coherent region at %px, dma %pad\n",
		 DRV_NAME, x->coherent, &x->coherent_dma);

	ret = devm_request_irq(&pdev->dev, x->irq, xk264_irq, 0,
			       dev_name(&pdev->dev), x);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ %d: %d\n",
			x->irq, ret);
		return ret;
	}
	dev_info(&pdev->dev, "%s: irq %d registered\n", DRV_NAME, x->irq);

	x->misc.minor = MISC_DYNAMIC_MINOR;
	x->misc.name = "xk264";
	x->misc.fops = &xk264_fops;
	x->misc.parent = &pdev->dev;

	ret = misc_register(&x->misc);
	if (ret) {
		dev_err(&pdev->dev, "misc_register failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, x);
	dev_info(&pdev->dev,
		 "xk264 H.264 encoder: base %pR, irq %d, dma %pad (%u KiB), /dev/%s minor %d\n",
		 res, x->irq, &x->coherent_dma, x->region_size >> 10,
		 x->misc.name, x->misc.minor);
	return 0;
}

static int xk264_remove(struct platform_device *pdev)
{
	struct xk264_dev *x = platform_get_drvdata(pdev);

	misc_deregister(&x->misc);
	return 0;
}

static const struct of_device_id xk264_of_match[] = {
	{ .compatible = "xk264,axi-encoder" },
	{ .compatible = "xlnx,xk264-axi-top-1.0" },
	{ }
};
MODULE_DEVICE_TABLE(of, xk264_of_match);

static struct platform_driver xk264_driver = {
	.probe	= xk264_probe,
	.remove	= xk264_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= xk264_of_match,
	},
};

module_platform_driver(xk264_driver);

MODULE_AUTHOR("xk264 integration");
MODULE_DESCRIPTION("xk264 H.264 AXI encoder driver (PetaLinux 2020.2)");
MODULE_LICENSE("GPL v2");
