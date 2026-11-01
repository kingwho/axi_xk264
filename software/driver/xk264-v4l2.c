// SPDX-License-Identifier: GPL-2.0
/*
 * xk264-v4l2 - standard V4L2 memory-to-memory encoder driver for the
 * Fudan VIPcore xk264 H.264 baseline encoder wrapped by
 * hdl/wrapper/xk264_axi_top.v.
 *
 * Model:
 *   OUTPUT  (V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)  : V4L2_PIX_FMT_YUV420M
 *   CAPTURE (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) : V4L2_PIX_FMT_H264
 *                                                  (Annex-B, start codes)
 *
 * The core only produces the macroblock layer CAVLC bits of each slice.
 * The driver adds everything around it so the capture queue delivers a
 * directly playable Annex-B stream:
 *   - SPS/PPS (repeated before every IDR)
 *   - slice headers (frame_num width derived from the GOP size)
 *   - the wrapper's 64-bit big-endian word packing is undone
 *   - emulation prevention bytes and Annex-B start codes
 *
 * The IP is a bus master: it fetches the input frame and writes the
 * bitstream / reconstruction frames to DDR over S_AXI_HP0.  The input
 * frame has to be stored in the wrapper's macroblock-tiled layout, so
 * the driver converts the planar YUV420 buffer in software.
 *
 * This module and the legacy xk264.ko (misc device /dev/xk264) drive the
 * same hardware and match the same device tree node, so only one of them
 * can be bound at a time.
 *
 * Target: PetaLinux 2020.2 / kernel 5.4, Zynq-7000.
 *
 * Kernel config requirements:
 *   CONFIG_VIDEO_V4L2          (v4l2 core + mem2mem)
 *   CONFIG_VIDEOBUF2_DMA_CONTIG
 *   CONFIG_VIDEOBUF2_V4L2
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/completion.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-memops.h>

#include <linux/version.h>

#include "xk264_regs.h"

#define DRV_NAME	"xk264-v4l2"

/*
 * Kernel config gates.  CONFIG_V4L2_MEM2MEM_DEV is a hidden symbol (it has
 * no prompt of its own), so it must be pulled in by enabling a mem2mem
 * consumer driver in the kernel config (e.g. CONFIG_VIDEO_MEM2MEM_DEINTERLACE),
 * otherwise struct v4l2_fh has no m2m_ctx member and the m2m core is not
 * built into v4l2.
 */
#if !IS_ENABLED(CONFIG_VIDEO_V4L2) || !IS_ENABLED(CONFIG_V4L2_MEM2MEM_DEV) || \
    !IS_ENABLED(CONFIG_VIDEOBUF2_V4L2) || \
    !IS_ENABLED(CONFIG_VIDEOBUF2_DMA_CONTIG)
#error "xk264-v4l2 needs CONFIG_VIDEO_V4L2, CONFIG_V4L2_MEM2MEM_DEV, CONFIG_VIDEOBUF2_V4L2 and CONFIG_VIDEOBUF2_DMA_CONTIG"
#endif

/* VFL_TYPE_VIDEO is the 5.10 name; kernel 5.4 still calls it VFL_TYPE_GRABBER */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
#define XK264_VFL_TYPE_VIDEO	VFL_TYPE_VIDEO
#else
#define XK264_VFL_TYPE_VIDEO	VFL_TYPE_GRABBER
#endif

/* ------------------------------------------------------------------ */
/* hardware buffer layout (same as the legacy misc driver)            */
/* ------------------------------------------------------------------ */

#define XK264_BS_SIZE		SZ_1M
#define XK264_MAX_MBS		((1920u / 16u) * (1080u / 16u))
#define XK264_MAX_YUV_SIZE	(XK264_MAX_MBS * 384u)	/* MB tiled, 384 B/MB */
#define XK264_MAX_BANK_SIZE	(XK264_MAX_MBS * 512u)

#define XK264_REGION_YUV_SIZE	((u32)PAGE_ALIGN(XK264_MAX_YUV_SIZE))
#define XK264_REGION_BANK_SIZE	((u32)PAGE_ALIGN(XK264_MAX_BANK_SIZE))
#define XK264_REGION_BS_SIZE	((u32)PAGE_ALIGN(XK264_BS_SIZE))

#define XK264_OFF_YUV		0x00000000u
#define XK264_OFF_BANK0		(XK264_OFF_YUV   + XK264_REGION_YUV_SIZE)
#define XK264_OFF_BANK1		(XK264_OFF_BANK0 + XK264_REGION_BANK_SIZE)
#define XK264_OFF_BS		(XK264_OFF_BANK1 + XK264_REGION_BANK_SIZE)
#define XK264_REGION_SIZE	(XK264_OFF_BS   + XK264_REGION_BS_SIZE)

#define XK264_DEFAULT_FPS	30
#define XK264_DEFAULT_GOP	30
#define XK264_DEFAULT_QP	28
#define XK264_DEFAULT_LEVEL	V4L2_MPEG_VIDEO_H264_LEVEL_4_0
#define XK264_DEFAULT_PROFILE	V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE
#define XK264_MIN_WIDTH		32
#define XK264_MIN_HEIGHT	32
#define XK264_MIN_CAP_SIZE	(64 * SZ_1K)	/* capture sizeimage floor */
#define XK264_TIMEOUT_MS	5000
#define XK264_NAL_BUF		64		/* SPS/PPS/slice header RBSP */

struct xk264_dev;
struct xk264_job;

/* ------------------------------------------------------------------ */
/* bit writer (MSB first) used for SPS / PPS / slice header RBSP      */
/* ------------------------------------------------------------------ */

struct xk264_bw {
	u8	*buf;
	u32	cap;		/* bytes */
	u32	nbits;
	bool	overflow;
};

static void xk264_bw_put(struct xk264_bw *w, u32 n, u32 v)
{
	u32 i;

	if (w->nbits + n > w->cap * 8) {
		w->overflow = true;
		return;
	}
	for (i = 0; i < n; i++) {
		u32 pos = w->nbits + i;

		if ((v >> (n - 1 - i)) & 1)
			w->buf[pos / 8] |= 0x80u >> (pos % 8);
	}
	w->nbits += n;
}

static void xk264_bw_ue(struct xk264_bw *w, u32 v)
{
	u32 s = v + 1, len = 0;

	while (s >> len)
		len++;
	xk264_bw_put(w, len - 1, 0);
	xk264_bw_put(w, len, s);
}

static void xk264_bw_se(struct xk264_bw *w, s32 v)
{
	if (v <= 0)
		xk264_bw_ue(w, (u32)(-2 * v));
	else
		xk264_bw_ue(w, (u32)(2 * v - 1));
}

/* rbsp_trailing_bits */
static void xk264_bw_trailing(struct xk264_bw *w)
{
	xk264_bw_put(w, 1, 1);
	while (w->nbits % 8)
		xk264_bw_put(w, 1, 0);
}

static void xk264_bw_init(struct xk264_bw *w, u8 *buf, u32 cap)
{
	memset(buf, 0, cap);
	w->buf = buf;
	w->cap = cap;
	w->nbits = 0;
	w->overflow = false;
}

/* ------------------------------------------------------------------ */
/* Annex-B output with emulation prevention                           */
/* ------------------------------------------------------------------ */

struct xk264_out {
	u8	*buf;
	size_t	cap;
	size_t	len;
	u32	acc;		/* pending bits, low accbits bits */
	u8	accbits;
	u8	zeros;		/* trailing zero bytes of the RBSP */
	bool	overflow;
};

/* start code / NAL header bytes: never touched by emulation prevention */
static void xk264_out_frame(struct xk264_out *o, u8 b)
{
	if (o->len >= o->cap) {
		o->overflow = true;
		return;
	}
	o->buf[o->len++] = b;
}

/* one RBSP byte, with emulation prevention insertion */
static void xk264_out_rbsp(struct xk264_out *o, u8 b)
{
	size_t need = 1 + ((o->zeros >= 2 && b <= 3) ? 1 : 0);

	if (o->len + need > o->cap) {
		o->overflow = true;
		return;
	}
	if (o->zeros >= 2 && b <= 3) {
		o->buf[o->len++] = 3;
		o->zeros = 0;
	}
	o->buf[o->len++] = b;
	if (b == 0)
		o->zeros++;
	else
		o->zeros = 0;
}

static void xk264_out_bit(struct xk264_out *o, u32 bit)
{
	o->acc = (o->acc << 1) | (bit & 1);
	if (++o->accbits == 8) {
		xk264_out_rbsp(o, (u8)o->acc);
		o->acc = 0;
		o->accbits = 0;
	}
}

/* append 8 bits at the current bit position */
static void xk264_out_u8(struct xk264_out *o, u8 b)
{
	o->acc = (o->acc << 8) | b;
	o->accbits += 8;
	while (o->accbits >= 8) {
		u8 bits = o->accbits - 8;

		xk264_out_rbsp(o, (u8)(o->acc >> bits));
		o->accbits = bits;
		o->acc &= (1u << bits) - 1;
	}
}

static void xk264_out_flush(struct xk264_out *o)
{
	if (o->accbits) {
		xk264_out_rbsp(o, (u8)(o->acc << (8 - o->accbits)));
		o->acc = 0;
		o->accbits = 0;
	}
}

static void xk264_out_bw(struct xk264_out *o, const struct xk264_bw *w)
{
	u32 i;

	for (i = 0; i < w->nbits; i++)
		xk264_out_bit(o, (w->buf[i >> 3] >> (7 - (i & 7))) & 1);
}

static void xk264_out_nal(struct xk264_out *o, u8 ref_idc, u8 type)
{
	xk264_out_frame(o, 0);
	xk264_out_frame(o, 0);
	xk264_out_frame(o, 0);
	xk264_out_frame(o, 1);
	xk264_out_frame(o, (u8)((ref_idc << 5) | type));
	o->zeros = 0;
	o->acc = 0;
	o->accbits = 0;
}

/* ------------------------------------------------------------------ */
/* SPS / PPS / slice header                                           */
/* ------------------------------------------------------------------ */

struct xk264_stream_cfg {
	u32 mb_w, mb_h;		/* coded size in macroblocks */
	u32 width, height;	/* requested (display) size */
	u32 qp;
	u32 level_idc;
	u32 fps;
	u32 log2_mfn;		/* log2_max_frame_num_minus4 + 4 */
};

static void xk264_make_sps(const struct xk264_stream_cfg *c, struct xk264_bw *w)
{
	u32 crop_r = (c->mb_w * 16 - c->width) / 2;
	u32 crop_b = (c->mb_h * 16 - c->height) / 2;

	xk264_bw_put(w, 8, 66);				/* profile_idc: baseline */
	xk264_bw_put(w, 8, 0);				/* constraint flags */
	xk264_bw_put(w, 8, c->level_idc);
	xk264_bw_ue(w, 0);				/* sps id */
	xk264_bw_ue(w, c->log2_mfn - 4);
	xk264_bw_ue(w, 2);				/* poc type */
	xk264_bw_ue(w, 1);				/* max_num_ref_frames */
	xk264_bw_put(w, 1, 0);				/* gaps_in_frame_num */
	xk264_bw_ue(w, c->mb_w - 1);
	xk264_bw_ue(w, c->mb_h - 1);
	xk264_bw_put(w, 1, 1);				/* frame_mbs_only */
	xk264_bw_put(w, 1, 1);				/* direct_8x8_inference */
	if (crop_r || crop_b) {
		xk264_bw_put(w, 1, 1);			/* frame_cropping */
		xk264_bw_ue(w, 0);			/* left */
		xk264_bw_ue(w, crop_r);			/* right */
		xk264_bw_ue(w, 0);			/* top */
		xk264_bw_ue(w, crop_b);			/* bottom */
	} else {
		xk264_bw_put(w, 1, 0);
	}
	/* VUI, timing information only */
	xk264_bw_put(w, 1, 1);				/* vui present */
	xk264_bw_put(w, 1, 0);				/* aspect_ratio_info */
	xk264_bw_put(w, 1, 0);				/* overscan_info */
	xk264_bw_put(w, 1, 0);				/* video_signal_type */
	xk264_bw_put(w, 1, 0);				/* chroma_loc_info */
	xk264_bw_put(w, 1, 1);				/* timing_info present */
	xk264_bw_put(w, 32, 1);				/* num_units_in_tick */
	xk264_bw_put(w, 32, 2 * c->fps);		/* time_scale */
	xk264_bw_put(w, 1, 1);				/* fixed_frame_rate */
	xk264_bw_put(w, 1, 0);				/* nal_hrd */
	xk264_bw_put(w, 1, 0);				/* vcl_hrd */
	xk264_bw_put(w, 1, 0);				/* pic_struct_present */
	xk264_bw_put(w, 1, 0);				/* bitstream_restriction */
	xk264_bw_trailing(w);
}

static void xk264_make_pps(const struct xk264_stream_cfg *c, struct xk264_bw *w)
{
	xk264_bw_ue(w, 0);				/* pps id */
	xk264_bw_ue(w, 0);				/* sps id */
	xk264_bw_put(w, 1, 0);				/* entropy: CAVLC */
	xk264_bw_put(w, 1, 0);				/* bottom_field_poc */
	xk264_bw_ue(w, 0);				/* slice groups */
	xk264_bw_ue(w, 0);				/* ref_idx_l0 */
	xk264_bw_ue(w, 0);				/* ref_idx_l1 */
	xk264_bw_put(w, 1, 0);				/* weighted_pred */
	xk264_bw_put(w, 2, 0);				/* weighted_bipred */
	xk264_bw_se(w, (s32)c->qp - 26);		/* pic_init_qp_minus26 */
	xk264_bw_se(w, 0);				/* pic_init_qs_minus26 */
	xk264_bw_se(w, 0);				/* chroma_qp_index_offset */
	xk264_bw_put(w, 1, 0);				/* deblocking control */
	xk264_bw_put(w, 1, 0);				/* constrained intra pred */
	xk264_bw_put(w, 1, 0);				/* redundant_pic_cnt */
	xk264_bw_trailing(w);
}

static void xk264_make_slice_header(const struct xk264_stream_cfg *c,
				    u32 frame_num, u32 idr_pic_id, bool idr,
				    struct xk264_bw *w)
{
	xk264_bw_ue(w, 0);				/* first_mb_in_slice */
	xk264_bw_ue(w, idr ? 7 : 5);			/* slice_type: I / P */
	xk264_bw_ue(w, 0);				/* pps id */
	xk264_bw_put(w, c->log2_mfn, frame_num);
	if (idr) {
		xk264_bw_ue(w, idr_pic_id);
		xk264_bw_put(w, 1, 0);			/* no_output_of_prior */
		xk264_bw_put(w, 1, 0);			/* long_term_reference */
	} else {
		xk264_bw_put(w, 1, 0);			/* ref_idx_override */
		xk264_bw_put(w, 1, 0);			/* ref_pic_list_mod */
		xk264_bw_put(w, 1, 0);			/* adaptive_ref_pic_mark */
	}
	xk264_bw_se(w, 0);				/* slice_qp_delta */
}

/* ------------------------------------------------------------------ */
/* planar I420 -> wrapper macroblock-tiled layout                     */
/* ------------------------------------------------------------------ */

#define XK264_BYTES_PER_MB	384

static void xk264_tile_frame(const u8 *y, const u8 *u, const u8 *v,
			     u32 width, u32 height, u8 *tiled)
{
	u32 mbw = (width + 15) / 16, mbh = (height + 15) / 16;
	u32 cw = width / 2, ch = height / 2;
	u32 mbx, mby, r, c;

	for (mby = 0; mby < mbh; mby++) {
		for (mbx = 0; mbx < mbw; mbx++) {
			u8 *mb = tiled + ((size_t)mby * mbw + mbx) *
					 XK264_BYTES_PER_MB;

			for (r = 0; r < 16; r++) {
				for (c = 0; c < 16; c++) {
					u32 sx = mbx * 16 + c;
					u32 sy = mby * 16 + r;

					if (sx >= width)
						sx = width - 1;
					if (sy >= height)
						sy = height - 1;
					mb[(r * 2 + (c >> 3)) * 8 + 7 - (c & 7)] =
						y[sy * width + sx];
				}
			}
			for (r = 0; r < 8; r++) {
				for (c = 0; c < 8; c++) {
					u32 sx = mbx * 8 + c;
					u32 sy = mby * 8 + r;
					u32 off;

					if (sx >= cw)
						sx = cw - 1;
					if (sy >= ch)
						sy = ch - 1;
					off = 256 + (r * 2 + (c >> 2)) * 8;
					mb[off + 7 - 2 * (c & 3)] = u[sy * cw + sx];
					mb[off + 6 - 2 * (c & 3)] = v[sy * cw + sx];
				}
			}
		}
	}
}

/*
 * The wrapper stores the bitstream with each 8-byte group byte reversed
 * (64-bit big-endian words).  Undo it in place; the last partial group
 * is reversed the same way.
 */
static void xk264_deroll(u8 *bs, u32 len)
{
	u32 off, i;

	for (off = 0; off + 8 <= len; off += 8) {
		for (i = 0; i < 4; i++) {
			u8 t = bs[off + i];

			bs[off + i] = bs[off + 7 - i];
			bs[off + 7 - i] = t;
		}
	}
	while (off < len) {
		u32 tail = len - off;

		for (i = 0; i < tail / 2; i++) {
			u8 t = bs[off + i];

			bs[off + i] = bs[off + tail - 1 - i];
			bs[off + tail - 1 - i] = t;
		}
		off += tail;
	}
}
/* ------------------------------------------------------------------ */
/* driver state                                                       */
/* ------------------------------------------------------------------ */

struct xk264_ctx {
	struct v4l2_fh			fh;
	struct v4l2_m2m_ctx		*m2m_ctx;
	struct xk264_dev		*dev;
	struct v4l2_ctrl_handler	hdl;
	struct v4l2_pix_format_mplane	src_fmt;	/* OUTPUT  */
	struct v4l2_pix_format_mplane	dst_fmt;	/* CAPTURE */
	struct v4l2_fract		timeperframe;

	struct v4l2_ctrl		*ctrl_gop;
	struct v4l2_ctrl		*ctrl_qp;
	struct v4l2_ctrl		*ctrl_profile;
	struct v4l2_ctrl		*ctrl_level;

	/* slice bookkeeping */
	u32				frame_count;
	u32				frames_since_idr;
	u32				idr_pic_id;
	bool				stopping;	/* teardown: no new jobs */
};

/* snapshot of the parameters of the frame being encoded */
struct xk264_job {
	struct xk264_ctx		*ctx;
	struct vb2_v4l2_buffer		*src;
	struct vb2_v4l2_buffer		*dst;
	u32				frame_num;
	u32				idr_pic_id;
	u32				qp;
	u32				level_idc;
	u32				fps;
	u32				log2_mfn;
	u32				width;
	u32				height;
	u32				mb_w;
	u32				mb_h;
	bool				idr;
};

struct xk264_dev {
	struct device			*dev;
	void __iomem			*base;
	int				irq;

	void				*coherent;
	dma_addr_t			coherent_dma;
	u32				region_size;

	struct v4l2_device		v4l2_dev;
	struct video_device		vdev;
	struct v4l2_m2m_dev		*m2m_dev;
	struct mutex			lock;	/* vdev + vb2 queue lock */

	struct xk264_job		job;
	struct completion		job_done;
	spinlock_t			job_lock;
	bool				job_active;	/* hw job in flight */
	bool				job_claimed;	/* someone is finishing it */
	u32				bs_len;
	u32				status_err;

	struct work_struct		done_work;
	struct delayed_work		timeout_work;
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
/* NAL / slice packaging of one finished frame                        */
/* ------------------------------------------------------------------ */

static u32 xk264_log2_mfn(u32 gop)
{
	u32 n = 4;

	while ((1u << n) <= gop - 1)
		n++;
	return n;
}

static u32 xk264_fps(struct xk264_ctx *ctx)
{
	u32 num = ctx->timeperframe.numerator;
	u32 den = ctx->timeperframe.denominator;
	u32 fps;

	if (!num || !den)
		return XK264_DEFAULT_FPS;
	fps = den / num;
	if (!fps)
		fps = 1;
	if (fps > 240)
		fps = 240;
	return fps;
}

static u32 xk264_level_idc(int level)
{
	switch (level) {
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_0: return 10;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_1: return 11;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_2: return 12;
	case V4L2_MPEG_VIDEO_H264_LEVEL_1_3: return 13;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_0: return 20;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_1: return 21;
	case V4L2_MPEG_VIDEO_H264_LEVEL_2_2: return 22;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_0: return 30;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_1: return 31;
	case V4L2_MPEG_VIDEO_H264_LEVEL_3_2: return 32;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_0: return 40;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_1: return 41;
	case V4L2_MPEG_VIDEO_H264_LEVEL_4_2: return 42;
	case V4L2_MPEG_VIDEO_H264_LEVEL_5_0: return 50;
	case V4L2_MPEG_VIDEO_H264_LEVEL_5_1: return 51;
	default: return 40;
	}
}

/*
 * Turn the encoder's raw macroblock-layer bits into a standard Annex-B
 * stream inside the capture buffer.  Returns the number of bytes written
 * or a negative error.
 */
static int xk264_build_stream(struct xk264_dev *x)
{
	struct xk264_job *j = &x->job;
	struct xk264_stream_cfg cfg;
	struct xk264_bw hdr;
	u8 hdrbuf[XK264_NAL_BUF];
	u8 sps[XK264_NAL_BUF];
	u8 pps[XK264_NAL_BUF];
	struct xk264_out o;
	const u8 *core = x->coherent + XK264_OFF_BS;
	u32 core_len = x->bs_len;
	u32 i;

	if (WARN_ON(core_len > XK264_BS_SIZE))
		return -EIO;

	cfg.mb_w = j->mb_w;
	cfg.mb_h = j->mb_h;
	cfg.width = j->width;
	cfg.height = j->height;
	cfg.qp = j->qp;
	cfg.level_idc = j->level_idc;
	cfg.fps = j->fps;
	cfg.log2_mfn = j->log2_mfn;

	o.buf = vb2_plane_vaddr(&j->dst->vb2_buf, 0);
	o.cap = vb2_plane_size(&j->dst->vb2_buf, 0);
	o.len = 0;
	o.acc = 0;
	o.accbits = 0;
	o.zeros = 0;
	o.overflow = false;
	if (!o.buf)
		return -EFAULT;

	if (j->idr) {
		xk264_bw_init(&hdr, sps, sizeof(sps));
		xk264_make_sps(&cfg, &hdr);
		if (hdr.overflow)
			return -EINVAL;
		xk264_out_nal(&o, 3, 7);
		xk264_out_bw(&o, &hdr);
		xk264_out_flush(&o);

		xk264_bw_init(&hdr, pps, sizeof(pps));
		xk264_make_pps(&cfg, &hdr);
		if (hdr.overflow)
			return -EINVAL;
		xk264_out_nal(&o, 3, 8);
		xk264_out_bw(&o, &hdr);
		xk264_out_flush(&o);
	}

	xk264_bw_init(&hdr, hdrbuf, sizeof(hdrbuf));
	xk264_make_slice_header(&cfg, j->frame_num, j->idr_pic_id, j->idr,
				&hdr);
	if (hdr.overflow)
		return -EINVAL;

	xk264_out_nal(&o, j->idr ? 3 : 2, j->idr ? 5 : 1);
	xk264_out_bw(&o, &hdr);
	for (i = 0; i < core_len; i++)
		xk264_out_u8(&o, core[i]);
	xk264_out_flush(&o);

	if (o.overflow)
		return -ENOSPC;

	return (int)o.len;
}

/* ------------------------------------------------------------------ */
/* hardware start / IRQ / completion                                  */
/* ------------------------------------------------------------------ */

static void xk264_hw_start(struct xk264_dev *x)
{
	struct xk264_job *j = &x->job;
	u32 cfg = (j->qp & 0x3f) | (j->idr ? BIT(6) : 0);

	xk264_write(x, XK264_REG_ISTAT, XK264_ISTAT_FRAME_DONE);	/* w1c */
	xk264_write(x, XK264_REG_CFG, cfg);
	xk264_write(x, XK264_REG_PIC, (j->mb_h << 16) | j->mb_w);
	xk264_write(x, XK264_REG_YUV, x->coherent_dma + XK264_OFF_YUV);
	xk264_write(x, XK264_REG_BANK0, x->coherent_dma + XK264_OFF_BANK0);
	xk264_write(x, XK264_REG_BANK1, x->coherent_dma + XK264_OFF_BANK1);
	xk264_write(x, XK264_REG_BSADDR, x->coherent_dma + XK264_OFF_BS);

	dma_wmb();

	/* frame mode (cfg[7] = 0), start, interrupt enable */
	xk264_write(x, XK264_REG_CTRL, XK264_CTRL_START | XK264_CTRL_IRQ_EN);
}

/* is a hardware job of this instance still in flight? */
static bool xk264_job_pending(struct xk264_dev *x, struct xk264_ctx *ctx)
{
	unsigned long flags;
	bool pending;

	spin_lock_irqsave(&x->job_lock, flags);
	pending = x->job_active && x->job.ctx == ctx;
	spin_unlock_irqrestore(&x->job_lock, flags);
	return pending;
}

/*
 * Called from stop_streaming/release (i.e. with the queue lock held) before
 * the buffers of this instance are handed back.  device_run() refuses to
 * start new jobs once ctx->stopping is set, so only the in-flight frame
 * has to be waited for.
 */
static void xk264_wait_job(struct xk264_dev *x, struct xk264_ctx *ctx)
{
	if (!xk264_job_pending(x, ctx))
		return;
	if (!wait_for_completion_timeout(&x->job_done,
					 msecs_to_jiffies(XK264_TIMEOUT_MS)))
		dev_warn(x->dev, "frame did not finish within %d ms\n",
			 XK264_TIMEOUT_MS);
}

/* hand both buffers back and let the m2m core schedule the next job */
static void xk264_finish_job(struct xk264_dev *x, bool error)
{
	struct xk264_job *j = &x->job;
	struct xk264_ctx *ctx = j->ctx;
	struct vb2_v4l2_buffer *src = j->src, *dst = j->dst;
	unsigned long flags;
	enum vb2_buffer_state state;

	if (!ctx || !src || !dst)
		return;

	dst->vb2_buf.timestamp = src->vb2_buf.timestamp;
	dst->sequence = src->sequence;
	dst->timecode = src->timecode;
	dst->field = src->field;
	dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			V4L2_BUF_FLAG_BFRAME);
	dst->flags |= j->idr ? V4L2_BUF_FLAG_KEYFRAME : V4L2_BUF_FLAG_PFRAME;

	if (!error) {
		int len = xk264_build_stream(x);

		if (len < 0) {
			dev_err(x->dev, "frame packaging failed: %d\n", len);
			error = true;
		} else {
			vb2_set_plane_payload(&dst->vb2_buf, 0, len);
		}
	}

	state = error ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE;
	v4l2_m2m_buf_done(src, state);
	v4l2_m2m_buf_done(dst, state);

	/*
	 * job_finish() may synchronously start the next job (possibly of
	 * another instance) from inside this call, so clear our bookkeeping
	 * afterwards and only if it still belongs to us.  The waiting
	 * instance is then released by the completion below.
	 */
	v4l2_m2m_job_finish(x->m2m_dev, ctx->m2m_ctx);

	spin_lock_irqsave(&x->job_lock, flags);
	if (x->job.ctx == ctx) {
		x->job.ctx = NULL;
		x->job.src = NULL;
		x->job.dst = NULL;
		x->job_active = false;
		x->job_claimed = false;
	}
	spin_unlock_irqrestore(&x->job_lock, flags);

	complete(&x->job_done);
}

static void xk264_done_work(struct work_struct *work)
{
	struct xk264_dev *x = container_of(work, struct xk264_dev, done_work);
	unsigned long flags;
	bool claimed = false;

	spin_lock_irqsave(&x->job_lock, flags);
	if (x->job_active && !x->job_claimed) {
		x->job_claimed = true;
		claimed = true;
	}
	spin_unlock_irqrestore(&x->job_lock, flags);

	if (!claimed)
		return;

	cancel_delayed_work_sync(&x->timeout_work);

	if (x->status_err) {
		dev_err(x->dev, "encode failed, status %#x\n", x->status_err);
		xk264_finish_job(x, true);
		return;
	}

	xk264_deroll(x->coherent + XK264_OFF_BS, x->bs_len);
	xk264_finish_job(x, false);
}

static void xk264_timeout_work(struct work_struct *work)
{
	struct xk264_dev *x = container_of(to_delayed_work(work),
					   struct xk264_dev, timeout_work);
	unsigned long flags;
	bool claimed = false;

	spin_lock_irqsave(&x->job_lock, flags);
	if (x->job_active && !x->job_claimed) {
		x->job_claimed = true;
		claimed = true;
	}
	spin_unlock_irqrestore(&x->job_lock, flags);

	if (!claimed)
		return;

	dev_err(x->dev,
		"encode timeout: status %#x, istat %#x, frmcnt %u (power cycle / reload the module to recover)\n",
		xk264_read(x, XK264_REG_STATUS),
		xk264_read(x, XK264_REG_ISTAT),
		xk264_read(x, XK264_REG_FRMCNT));
	xk264_finish_job(x, true);
}

static irqreturn_t xk264_irq(int irq, void *data)
{
	struct xk264_dev *x = data;
	u32 istat = xk264_read(x, XK264_REG_ISTAT);

	if (!(istat & XK264_ISTAT_FRAME_DONE))
		return IRQ_NONE;

	xk264_write(x, XK264_REG_ISTAT, XK264_ISTAT_FRAME_DONE);	/* w1c */
	x->bs_len = xk264_read(x, XK264_REG_BSLEN);
	x->status_err = xk264_read(x, XK264_REG_STATUS) &
			(XK264_STATUS_RD_ERR | XK264_STATUS_WR_ERR);

	dev_dbg(x->dev, "irq: istat %#x, bs_len %u, status_err %#x\n",
		istat, x->bs_len, x->status_err);

	schedule_work(&x->done_work);
	return IRQ_HANDLED;
}
/* ------------------------------------------------------------------ */
/* formats                                                            */
/* ------------------------------------------------------------------ */

static void xk264_set_src_planes(struct v4l2_pix_format_mplane *p)
{
	u32 w = p->width, h = p->height;

	p->pixelformat = V4L2_PIX_FMT_YUV420M;
	p->field = V4L2_FIELD_NONE;
	p->num_planes = 3;
	p->plane_fmt[0].bytesperline = w;
	p->plane_fmt[0].sizeimage = w * h;
	p->plane_fmt[1].bytesperline = w / 2;
	p->plane_fmt[1].sizeimage = (w / 2) * (h / 2);
	p->plane_fmt[2].bytesperline = w / 2;
	p->plane_fmt[2].sizeimage = (w / 2) * (h / 2);
}

static void xk264_default_src_fmt(struct xk264_ctx *ctx)
{
	struct v4l2_pix_format_mplane *p = &ctx->src_fmt;

	memset(p, 0, sizeof(*p));
	p->width = 640;
	p->height = 480;
	p->colorspace = V4L2_COLORSPACE_REC709;
	p->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	p->quantization = V4L2_QUANTIZATION_DEFAULT;
	xk264_set_src_planes(p);
}

static void xk264_default_dst_fmt(struct xk264_ctx *ctx)
{
	struct v4l2_pix_format_mplane *p = &ctx->dst_fmt;

	memset(p, 0, sizeof(*p));
	p->width = ALIGN(640, 16);
	p->height = ALIGN(480, 16);
	p->colorspace = V4L2_COLORSPACE_REC709;
	p->pixelformat = V4L2_PIX_FMT_H264;
	p->field = V4L2_FIELD_NONE;
	p->num_planes = 1;
	p->plane_fmt[0].bytesperline = 0;
	p->plane_fmt[0].sizeimage = XK264_BS_SIZE;
}

static void xk264_try_fmt_out(struct xk264_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *p = &f->fmt.pix_mp;
	u32 w, h;

	w = clamp_t(u32, round_down(p->width, 2), XK264_MIN_WIDTH, 1920u);
	h = clamp_t(u32, round_down(p->height, 2), XK264_MIN_HEIGHT, 1088u);
	while ((w / 16) * (h / 16) > XK264_MAX_MBS && h >= XK264_MIN_HEIGHT + 16)
		h -= 16;
	while ((w / 16) * (h / 16) > XK264_MAX_MBS && w >= XK264_MIN_WIDTH + 16)
		w -= 16;

	p->width = w;
	p->height = h;
	if (p->colorspace == V4L2_COLORSPACE_DEFAULT)
		p->colorspace = V4L2_COLORSPACE_REC709;
	xk264_set_src_planes(p);
}

static void xk264_try_fmt_cap(struct xk264_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *p = &f->fmt.pix_mp;

	p->pixelformat = V4L2_PIX_FMT_H264;
	p->field = V4L2_FIELD_NONE;
	p->num_planes = 1;
	p->plane_fmt[0].bytesperline = 0;
	if (!p->plane_fmt[0].sizeimage)
		p->plane_fmt[0].sizeimage = XK264_BS_SIZE;
	p->plane_fmt[0].sizeimage = clamp_t(u32, p->plane_fmt[0].sizeimage,
					   XK264_MIN_CAP_SIZE, XK264_BS_SIZE);
	p->width = ALIGN(ctx->src_fmt.width, 16);
	p->height = ALIGN(ctx->src_fmt.height, 16);
	if (p->colorspace == V4L2_COLORSPACE_DEFAULT)
		p->colorspace = V4L2_COLORSPACE_REC709;
}

/* ------------------------------------------------------------------ */
/* vb2 queue ops                                                      */
/* ------------------------------------------------------------------ */

static struct v4l2_pix_format_mplane *xk264_queue_fmt(struct vb2_queue *vq)
{
	struct xk264_ctx *ctx = vb2_get_drv_priv(vq);

	return (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) ?
		&ctx->src_fmt : &ctx->dst_fmt;
}

static int xk264_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct v4l2_pix_format_mplane *f = xk264_queue_fmt(vq);
	unsigned int i;

	if (*nplanes) {
		if (*nplanes != f->num_planes)
			return -EINVAL;
		for (i = 0; i < *nplanes; i++)
			if (sizes[i] < f->plane_fmt[i].sizeimage)
				return -EINVAL;
		return 0;
	}

	*nplanes = f->num_planes;
	for (i = 0; i < f->num_planes; i++) {
		sizes[i] = f->plane_fmt[i].sizeimage;
		alloc_devs[i] = vq->dev;
	}
	return 0;
}

static int xk264_buf_prepare(struct vb2_buffer *vb)
{
	struct v4l2_pix_format_mplane *f = xk264_queue_fmt(vb->vb2_queue);
	unsigned int i;

	for (i = 0; i < f->num_planes; i++)
		if (vb2_plane_size(vb, i) < f->plane_fmt[i].sizeimage)
			return -EINVAL;

	if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		for (i = 0; i < f->num_planes; i++)
			vb2_set_plane_payload(vb, i, f->plane_fmt[i].sizeimage);
	return 0;
}

static int xk264_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct xk264_ctx *ctx = vb2_get_drv_priv(vq);

	ctx->stopping = false;
	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* a new stream always starts with an IDR */
		ctx->frame_count = 0;
		ctx->frames_since_idr = 0;
		ctx->idr_pic_id = 0;
	}
	return 0;
}

static void xk264_stop_streaming(struct vb2_queue *vq)
{
	struct xk264_ctx *ctx = vb2_get_drv_priv(vq);
	struct xk264_dev *x = ctx->dev;
	struct vb2_v4l2_buffer *buf;

	/* no new job may be started for this instance ... */
	ctx->stopping = true;
	/* ... and the in-flight frame has to finish before its buffers go */
	xk264_wait_job(x, ctx);

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
	}
}

/*
 * buf_queue adapter: vb2_ops.buf_queue expects void (*)(struct vb2_buffer *),
 * but this kernel's v4l2_m2m_buf_queue() takes (struct v4l2_m2m_ctx *,
 * struct vb2_v4l2_buffer *).  Resolve the m2m context from the queue's
 * drv_priv (set in xk264_queue_init) and forward the call.
 */
static void xk264_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct xk264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->m2m_ctx, vbuf);
}

static const struct vb2_ops xk264_qops = {
	.queue_setup	= xk264_queue_setup,
	.buf_prepare	= xk264_buf_prepare,
	.buf_queue	= xk264_buf_queue,
	.start_streaming = xk264_start_streaming,
	.stop_streaming	= xk264_stop_streaming,
};

/* ------------------------------------------------------------------ */
/* mem2mem device_run                                                 */
/* ------------------------------------------------------------------ */

static void xk264_device_run(void *priv)
{
	struct xk264_ctx *ctx = priv;
	struct xk264_dev *x = ctx->dev;
	struct xk264_job *j = &x->job;
	struct vb2_v4l2_buffer *src, *dst;
	struct vb2_buffer *svb;
	unsigned long flags;
	u32 gop;

	src = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	if (!src || !dst) {
		dev_err(x->dev, "device_run without buffers\n");
		return;
	}

	/*
	 * The instance is being torn down (stop_streaming/release): do not
	 * start the hardware, just hand the buffers back so the queues drain.
	 */
	if (ctx->stopping) {
		v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(x->m2m_dev, ctx->m2m_ctx);
		return;
	}

	/* the job owns both buffers from here on */
	v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	svb = &src->vb2_buf;

	gop = ctx->ctrl_gop->val;
	if (!gop)
		gop = 1;

	j->ctx = ctx;
	j->src = src;
	j->dst = dst;
	j->idr = (ctx->frame_count % gop) == 0;
	if (j->idr) {
		if (ctx->frames_since_idr)
			ctx->idr_pic_id ^= 1;
		ctx->frames_since_idr = 0;
		j->frame_num = 0;
	} else {
		j->frame_num = ctx->frames_since_idr;
	}
	j->idr_pic_id = ctx->idr_pic_id;
	ctx->frames_since_idr++;
	ctx->frame_count++;

	j->qp = ctx->ctrl_qp->val;
	j->level_idc = xk264_level_idc(ctx->ctrl_level->val);
	j->fps = xk264_fps(ctx);
	j->log2_mfn = xk264_log2_mfn(gop);
	j->width = ctx->src_fmt.width;
	j->height = ctx->src_fmt.height;
	j->mb_w = (j->width + 15) / 16;
	j->mb_h = (j->height + 15) / 16;

	xk264_tile_frame(vb2_plane_vaddr(svb, 0), vb2_plane_vaddr(svb, 1),
			 vb2_plane_vaddr(svb, 2), j->width, j->height,
			 x->coherent + XK264_OFF_YUV);

	reinit_completion(&x->job_done);

	spin_lock_irqsave(&x->job_lock, flags);
	x->job_active = true;
	x->job_claimed = false;
	x->status_err = 0;
	spin_unlock_irqrestore(&x->job_lock, flags);

	if (xk264_read(x, XK264_REG_STATUS) & XK264_STATUS_BUSY) {
		dev_err(x->dev,
			"encoder still busy, status %#x (power cycle / reload to recover)\n",
			xk264_read(x, XK264_REG_STATUS));
		x->status_err = 1;
		schedule_work(&x->done_work);
		return;
	}

	queue_delayed_work(system_wq, &x->timeout_work,
			   msecs_to_jiffies(XK264_TIMEOUT_MS));
	xk264_hw_start(x);
}

static const struct v4l2_m2m_ops xk264_m2m_ops = {
	.device_run	= xk264_device_run,
};

static int xk264_queue_init(void *priv, struct vb2_queue *src_vq,
			    struct vb2_queue *dst_vq)
{
	struct xk264_ctx *ctx = priv;
	struct xk264_dev *x = ctx->dev;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &xk264_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->dev = x->dev;
	src_vq->lock = &x->lock;
	ret = vb2_queue_init(src_vq);
	if (ret) {
		dev_err(x->dev, "queue_init: src vb2_queue_init failed: %d\n", ret);
		return ret;
	}

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &xk264_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->dev = x->dev;
	dst_vq->lock = &x->lock;

	ret = vb2_queue_init(dst_vq);
	if (ret)
		dev_err(x->dev, "queue_init: dst vb2_queue_init failed: %d\n", ret);
	return ret;
}
/* ------------------------------------------------------------------ */
/* controls                                                           */
/* ------------------------------------------------------------------ */

static const struct v4l2_ctrl_ops xk264_ctrl_ops = {
};

/* ------------------------------------------------------------------ */
/* ioctls                                                             */
/* ------------------------------------------------------------------ */

static int xk264_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct xk264_ctx *ctx = priv;
	struct xk264_dev *x = ctx->dev;

	strscpy(cap->driver, DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, "xk264 H.264 encoder", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(x->dev));
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int xk264_enum_fmt_out(struct file *file, void *priv,
			      struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_YUV420M;
	return 0;
}

static int xk264_enum_fmt_cap(struct file *file, void *priv,
			      struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_H264;
	return 0;
}

static int xk264_g_fmt_out(struct file *file, void *priv,
			   struct v4l2_format *f)
{
	struct xk264_ctx *ctx = priv;

	f->fmt.pix_mp = ctx->src_fmt;
	return 0;
}

static int xk264_g_fmt_cap(struct file *file, void *priv,
			   struct v4l2_format *f)
{
	struct xk264_ctx *ctx = priv;

	f->fmt.pix_mp = ctx->dst_fmt;
	return 0;
}

static int xk264_try_fmt_out_ioctl(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct xk264_ctx *ctx = priv;

	xk264_try_fmt_out(ctx, f);
	return 0;
}

static int xk264_try_fmt_cap_ioctl(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct xk264_ctx *ctx = priv;

	xk264_try_fmt_cap(ctx, f);
	return 0;
}

static int xk264_s_fmt_out(struct file *file, void *priv,
			   struct v4l2_format *f)
{
	struct xk264_ctx *ctx = priv;
	struct vb2_queue *vq;
	unsigned int mbs;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (vq && vb2_is_busy(vq))
		return -EBUSY;

	dev_info(ctx->dev->dev,
		 "s_fmt_out(in): %ux%u pix=%#x num_planes=%u\n",
		 f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		 f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.num_planes);

	xk264_try_fmt_out(ctx, f);

	mbs = (f->fmt.pix_mp.width / 16) * (f->fmt.pix_mp.height / 16);
	dev_info(ctx->dev->dev,
		 "s_fmt_out(out): %ux%u mbs=%u max=%u\n",
		 f->fmt.pix_mp.width, f->fmt.pix_mp.height, mbs, XK264_MAX_MBS);

	if (mbs > XK264_MAX_MBS)
		return -EINVAL;

	ctx->src_fmt = f->fmt.pix_mp;
	/* keep the capture queue's coded size in sync */
	ctx->dst_fmt.width = ALIGN(ctx->src_fmt.width, 16);
	ctx->dst_fmt.height = ALIGN(ctx->src_fmt.height, 16);
	return 0;
}

static int xk264_s_fmt_cap(struct file *file, void *priv,
			   struct v4l2_format *f)
{
	struct xk264_ctx *ctx = priv;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (vq && vb2_is_busy(vq))
		return -EBUSY;

	xk264_try_fmt_cap(ctx, f);
	ctx->dst_fmt = f->fmt.pix_mp;
	return 0;
}

static int xk264_g_parm(struct file *file, void *priv,
			struct v4l2_streamparm *p)
{
	struct xk264_ctx *ctx = priv;

	if (p->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;
	p->parm.output.timeperframe = ctx->timeperframe;
	return 0;
}

static int xk264_s_parm(struct file *file, void *priv,
			struct v4l2_streamparm *p)
{
	struct xk264_ctx *ctx = priv;
	struct v4l2_fract *tpf = &p->parm.output.timeperframe;

	if (p->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;
	if (!tpf->numerator || !tpf->denominator)
		return -EINVAL;

	ctx->timeperframe = *tpf;
	p->parm.output.timeperframe = ctx->timeperframe;
	return 0;
}

/*
 * GStreamer's v4l2h264enc builds its sink caps from VIDIOC_ENUM_FRAMESIZES
 * and VIDIOC_ENUM_FRAMEINTERVALS.  Without these the caps template is empty
 * and negotiation fails (not-negotiated).  The same range covers both the
 * raw OUTPUT and the coded CAPTURE queue (the coded size is just the OUTPUT
 * size rounded up to the next 16-pixel boundary).
 */
static int xk264_enum_framesizes(struct file *file, void *priv,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;
	if (fsize->pixel_format != V4L2_PIX_FMT_YUV420M &&
	    fsize->pixel_format != V4L2_PIX_FMT_H264)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = XK264_MIN_WIDTH;
	fsize->stepwise.max_width = 1920;
	fsize->stepwise.step_width = 2;
	fsize->stepwise.min_height = XK264_MIN_HEIGHT;
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
	/* time-per-frame range: 1/240 s .. 1/1 s (1 .. 240 fps).
	 * step must be <= (max-min) or the range collapses to a single
	 * value and GStreamer rejects the requested framerate. */
	fival->stepwise.min.numerator = 1;
	fival->stepwise.min.denominator = 240;
	fival->stepwise.max.numerator = 1;
	fival->stepwise.max.denominator = 1;
	fival->stepwise.step.numerator = 1;
	fival->stepwise.step.denominator = 240;
	return 0;
}

/*
 * 5.4 has no drain state machine in the m2m core (there is no
 * v4l2_m2m_ioctl_encoder_cmd), and this hardware has no internal frame
 * queue: every OUTPUT buffer is encoded synchronously in device_run, so
 * once all of them have been processed all CAPTURE buffers are ready.
 * Accepting STOP is therefore enough for GStreamer's EOS handling.
 */
static int xk264_encoder_cmd(struct file *file, void *priv,
			     struct v4l2_encoder_cmd *ec)
{
	if (ec->cmd != V4L2_ENC_CMD_STOP)
		return -EINVAL;
	return 0;
}

static const struct v4l2_ioctl_ops xk264_ioctl_ops = {
	.vidioc_querycap		= xk264_querycap,
	/* 5.4 has no *_mplane enum_fmt variants; the same field serves both */
	.vidioc_enum_fmt_vid_cap	= xk264_enum_fmt_cap,
	.vidioc_enum_fmt_vid_out	= xk264_enum_fmt_out,
	.vidioc_g_fmt_vid_cap_mplane	= xk264_g_fmt_cap,
	.vidioc_g_fmt_vid_out_mplane	= xk264_g_fmt_out,
	.vidioc_try_fmt_vid_cap_mplane	= xk264_try_fmt_cap_ioctl,
	.vidioc_try_fmt_vid_out_mplane	= xk264_try_fmt_out_ioctl,
	.vidioc_s_fmt_vid_cap_mplane	= xk264_s_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane	= xk264_s_fmt_out,
	.vidioc_g_parm			= xk264_g_parm,
	.vidioc_s_parm			= xk264_s_parm,
	.vidioc_enum_framesizes		= xk264_enum_framesizes,
	.vidioc_enum_frameintervals	= xk264_enum_frameintervals,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
	.vidioc_encoder_cmd		= xk264_encoder_cmd,
	.vidioc_try_encoder_cmd		= xk264_encoder_cmd,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* ------------------------------------------------------------------ */
/* file operations                                                    */
/* ------------------------------------------------------------------ */

static int xk264_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct xk264_dev *x = video_get_drvdata(vdev);
	struct xk264_ctx *ctx;
	int ret;

	dev_dbg(x->dev, "open: enter (m2m_dev=%p)\n", x->m2m_dev);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = x;
	ctx->timeperframe.numerator = 1;
	ctx->timeperframe.denominator = XK264_DEFAULT_FPS;

	v4l2_fh_init(&ctx->fh, vdev);
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	v4l2_ctrl_handler_init(&ctx->hdl, 4);
	ctx->ctrl_gop = v4l2_ctrl_new_std(&ctx->hdl, &xk264_ctrl_ops,
			V4L2_CID_MPEG_VIDEO_GOP_SIZE, 1, 300, 1,
			XK264_DEFAULT_GOP);
	if (!ctx->ctrl_gop)
		dev_err(x->dev, "open: ctrl GOP_SIZE failed (hdl.error=%d)\n",
			ctx->hdl.error);
	ctx->ctrl_qp = v4l2_ctrl_new_std(&ctx->hdl, &xk264_ctrl_ops,
			V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 0, 51, 1,
			XK264_DEFAULT_QP);
	if (!ctx->ctrl_qp)
		dev_err(x->dev, "open: ctrl H264_MIN_QP failed (hdl.error=%d)\n",
			ctx->hdl.error);
	ctx->ctrl_profile = v4l2_ctrl_new_std_menu(&ctx->hdl, &xk264_ctrl_ops,
			V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE, 0,
			XK264_DEFAULT_PROFILE);
	if (!ctx->ctrl_profile)
		dev_err(x->dev, "open: ctrl H264_PROFILE failed (hdl.error=%d)\n",
			ctx->hdl.error);
	ctx->ctrl_level = v4l2_ctrl_new_std_menu(&ctx->hdl, &xk264_ctrl_ops,
			V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			V4L2_MPEG_VIDEO_H264_LEVEL_5_1, 0,
			XK264_DEFAULT_LEVEL);
	if (!ctx->ctrl_level)
		dev_err(x->dev, "open: ctrl H264_LEVEL failed (hdl.error=%d)\n",
			ctx->hdl.error);
	if (ctx->hdl.error) {
		ret = ctx->hdl.error;
		dev_err(x->dev, "open: ctrl handler init failed: %d\n", ret);
		goto err_hdl;
	}
	ctx->fh.ctrl_handler = &ctx->hdl;

	xk264_default_src_fmt(ctx);
	xk264_default_dst_fmt(ctx);

	ctx->m2m_ctx = v4l2_m2m_ctx_init(x->m2m_dev, ctx, xk264_queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		dev_err(x->dev, "open: v4l2_m2m_ctx_init failed: %d "
			"(m2m_dev=%p)\n", ret, x->m2m_dev);
		goto err_hdl;
	}
	/* v4l2_m2m_fop_poll()/mmap() resolve the queues through the fh */
	ctx->fh.m2m_ctx = ctx->m2m_ctx;

	dev_dbg(x->dev, "open by pid %d\n", current->pid);
	return 0;

err_hdl:
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int xk264_release(struct file *file)
{
	struct xk264_ctx *ctx =
		container_of(file->private_data, struct xk264_ctx, fh);
	struct xk264_dev *x = ctx->dev;

	/* do not free the context while a frame still uses it */
	ctx->stopping = true;
	xk264_wait_job(x, ctx);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->hdl);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations xk264_fops = {
	.owner		= THIS_MODULE,
	.open		= xk264_open,
	.release	= xk264_release,
	.unlocked_ioctl	= video_ioctl2,
	.poll		= v4l2_m2m_fop_poll,
	.mmap		= v4l2_m2m_fop_mmap,
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

	x = devm_kzalloc(&pdev->dev, sizeof(*x), GFP_KERNEL);
	if (!x)
		return -ENOMEM;

	x->dev = &pdev->dev;
	mutex_init(&x->lock);
	spin_lock_init(&x->job_lock);
	init_completion(&x->job_done);
	INIT_WORK(&x->done_work, xk264_done_work);
	INIT_DELAYED_WORK(&x->timeout_work, xk264_timeout_work);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no IORESOURCE_MEM resource\n");
		return -ENODEV;
	}
	x->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(x->base))
		return PTR_ERR(x->base);

	idr = xk264_read(x, XK264_REG_IDR);
	if (idr != XK264_IDR_MAGIC) {
		dev_err(&pdev->dev, "bad ID register %#x (expected %#x)\n",
			idr, XK264_IDR_MAGIC);
		return -ENODEV;
	}

	x->irq = platform_get_irq(pdev, 0);
	if (x->irq < 0)
		return x->irq;

	ret = devm_request_irq(&pdev->dev, x->irq, xk264_irq, 0,
			       dev_name(&pdev->dev), x);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ %d: %d\n",
			x->irq, ret);
		return ret;
	}

	x->region_size = XK264_REGION_SIZE;
	x->coherent = dmam_alloc_coherent(&pdev->dev, x->region_size,
					  &x->coherent_dma, GFP_KERNEL);
	if (!x->coherent) {
		dev_err(&pdev->dev,
			"dmam_alloc_coherent(%u bytes) failed: check CMA size\n",
			x->region_size);
		return -ENOMEM;
	}

	ret = v4l2_device_register(&pdev->dev, &x->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "v4l2_device_register failed: %d\n", ret);
		return ret;
	}

	x->m2m_dev = v4l2_m2m_init(&xk264_m2m_ops);
	if (IS_ERR(x->m2m_dev)) {
		ret = PTR_ERR(x->m2m_dev);
		dev_err(&pdev->dev, "v4l2_m2m_init failed: %d\n", ret);
		goto err_v4l2;
	}

	strscpy(x->vdev.name, "xk264-encoder", sizeof(x->vdev.name));
	x->vdev.v4l2_dev = &x->v4l2_dev;
	x->vdev.fops = &xk264_fops;
	x->vdev.ioctl_ops = &xk264_ioctl_ops;
	x->vdev.release = video_device_release_empty;
	x->vdev.lock = &x->lock;
	x->vdev.vfl_dir = VFL_DIR_M2M;
	x->vdev.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(&x->vdev, x);

	ret = video_register_device(&x->vdev, XK264_VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&pdev->dev, "video_register_device failed: %d\n", ret);
		goto err_m2m;
	}

	platform_set_drvdata(pdev, x);
	dev_info(&pdev->dev,
		 "xk264 H.264 encoder: /dev/video%d, base %pR, irq %d, dma %pad (%u KiB)\n",
		 x->vdev.num, res, x->irq, &x->coherent_dma,
		 x->region_size >> 10);
	return 0;

err_m2m:
	v4l2_m2m_release(x->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&x->v4l2_dev);
	return ret;
}

static int xk264_remove(struct platform_device *pdev)
{
	struct xk264_dev *x = platform_get_drvdata(pdev);

	if (!x)
		return 0;

	cancel_delayed_work_sync(&x->timeout_work);
	cancel_work_sync(&x->done_work);
	video_unregister_device(&x->vdev);
	v4l2_m2m_release(x->m2m_dev);
	v4l2_device_unregister(&x->v4l2_dev);
	return 0;
}

static const struct of_device_id xk264_of_match[] = {
	{ .compatible = "xk264,axi-encoder" },
	{ .compatible = "xlnx,xk264-axi-top-1.0" },
	{ .compatible = "xk264,v4l2-encoder" },
	{ }
};
MODULE_DEVICE_TABLE(of, xk264_of_match);

static struct platform_driver xk264_v4l2_driver = {
	.probe	= xk264_probe,
	.remove	= xk264_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= xk264_of_match,
	},
};

module_platform_driver(xk264_v4l2_driver);

MODULE_AUTHOR("xk264 integration");
MODULE_DESCRIPTION("xk264 H.264 encoder V4L2 mem2mem driver (PetaLinux 2020.2)");
MODULE_LICENSE("GPL v2");
