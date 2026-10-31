// SPDX-License-Identifier: GPL-2.0
/*
 * xk264_app - application demo for the xk264 H.264 hardware encoder.
 *
 * Configurable resolution / frame rate / GOP / QP, encodes a raw I420
 * input file (or a built-in rolling color-bar test pattern) through
 * /dev/xk264 and writes a directly playable Annex-B .h264 stream.
 *
 * The core emits only the macroblock-layer CAVLC bits of each slice.
 * This application adds all surrounding syntax in software so standard
 * players/decoders accept the output:
 *   - SPS (baseline, crop for non multiple-of-16 sizes, VUI timing for
 *     the requested frame rate) and PPS, repeated before every IDR
 *   - slice headers (frame_num width derived from the GOP)
 *   - emulation-prevention bytes and Annex-B start codes
 *
 * It also undoes the wrapper's 64-bit big-endian word packing: the
 * bitstream in DDR is stored with each 8-byte group byte-reversed.
 *
 * Build (host, for offline packaging tests):
 *     make
 * Build (target, PetaLinux 2020.2 / Zynq-7000):
 *     make CROSS=arm-linux-gnueabihf-
 *
 * Run on target:
 *     ./xk264_app -W 640 -H 480 -f 30 -g 30 -q 28 -i in.yuv -o out.h264
 *     ffplay out.h264
 *
 * Offline (no hardware, package simulation dumps bs_frame<N>.bin):
 *     ./xk264_app --sim-dir /tmp/xk264_gop -W 640 -H 480 -f 30 -g 3
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

/* ---- driver interface (must match software/driver/xk264.c) ---- */

struct xk264_param {
	uint16_t mb_x_total;
	uint16_t mb_y_total;
	uint8_t  qp;
	uint8_t  reserved[3];
};

struct xk264_encode {
	uint32_t flags;
	uint32_t bs_len;
	uint32_t timeout_ms;
	uint32_t reserved;
};

struct xk264_info {
	uint32_t hw_id;
	uint32_t max_mbs;
	uint32_t mmap_off[2];
	uint32_t mmap_len[2];
};

#define XK264_ENC_INTRA    0x1
#define XK264_IOC_MAGIC    'x'
#define XK264_IOC_G_INFO   _IOR(XK264_IOC_MAGIC, 0x00, struct xk264_info)
#define XK264_IOC_S_PARAM  _IOW(XK264_IOC_MAGIC, 0x01, struct xk264_param)
#define XK264_IOC_ENCODE   _IOWR(XK264_IOC_MAGIC, 0x02, struct xk264_encode)

#define DEV_PATH   "/dev/xk264"
#define BYTES_PER_MB 384

/* ---- options ---- */

struct opts {
	int   width, height;
	int   fps;
	int   gop;
	int   qp;
	int   nframes;
	int   timeout_ms;
	int   level_idc;
	int   pace;
	const char *input;
	const char *output;
	const char *sim_dir;   /* offline mode */
};

static struct opts o = {
	.width = 640, .height = 480,
	.fps = 30, .gop = 30, .qp = 28,
	.nframes = 0, .timeout_ms = 10000,
	.level_idc = 40, .pace = 1,
	.input = NULL, .output = "out.h264", .sim_dir = NULL,
};

static void die(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	exit(1);
}

static void usage(void)
{
	printf(
"usage: xk264_app [options]\n"
"  -W, --width N     picture width in pixels  (default 640)\n"
"  -H, --height N    picture height in pixels (default 480)\n"
"  -f, --fps N       frame rate, also written to VUI (default 30)\n"
"  -g, --gop N       IDR interval in frames, 1 = all-intra (default 30)\n"
"  -q, --qp N        quantization parameter 0..51 (default 28)\n"
"  -n, --frames N    number of frames to encode (default: input EOF,\n"
"                    60 for the built-in test pattern)\n"
"  -i, --input FILE  raw I420 planar input file\n"
"  -o, --output FILE output Annex-B H.264 file (default out.h264)\n"
"  -t, --timeout MS  per-frame encode timeout (default 10000)\n"
"      --level IDC   SPS level_idc (default 40)\n"
"      --no-pace     encode flat out, don't sleep between frames\n"
"      --sim-dir DIR offline: package bs_frame<N>.bin dumps, no hardware\n"
"  -h, --help        this help\n");
	exit(0);
}

static void parse_opts(int argc, char **argv)
{
	static const struct option lo[] = {
		{ "width",   required_argument, NULL, 'W' },
		{ "height",  required_argument, NULL, 'H' },
		{ "fps",     required_argument, NULL, 'f' },
		{ "gop",     required_argument, NULL, 'g' },
		{ "qp",      required_argument, NULL, 'q' },
		{ "frames",  required_argument, NULL, 'n' },
		{ "input",   required_argument, NULL, 'i' },
		{ "output",  required_argument, NULL, 'o' },
		{ "timeout", required_argument, NULL, 't' },
		{ "level",   required_argument, NULL,  1  },
		{ "no-pace", no_argument,       NULL,  2  },
		{ "sim-dir", required_argument, NULL,  3  },
		{ "help",    no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int c;

	while ((c = getopt_long(argc, argv, "W:H:f:g:q:n:i:o:t:h", lo, NULL)) != -1) {
		switch (c) {
		case 'W': o.width  = atoi(optarg); break;
		case 'H': o.height = atoi(optarg); break;
		case 'f': o.fps    = atoi(optarg); break;
		case 'g': o.gop    = atoi(optarg); break;
		case 'q': o.qp     = atoi(optarg); break;
		case 'n': o.nframes = atoi(optarg); break;
		case 'i': o.input  = optarg; break;
		case 'o': o.output = optarg; break;
		case 't': o.timeout_ms = atoi(optarg); break;
		case 1:   o.level_idc = atoi(optarg); break;
		case 2:   o.pace = 0; break;
		case 3:   o.sim_dir = optarg; break;
		case 'h': default: usage();
		}
	}
	if (o.width <= 0 || o.height <= 0)
		die("bad picture size %dx%d\n", o.width, o.height);
	if (o.fps <= 0 || o.fps > 240)
		die("bad fps %d\n", o.fps);
	if (o.gop <= 0)
		die("bad gop %d\n", o.gop);
	if (o.qp < 0 || o.qp > 51)
		die("bad qp %d\n", o.qp);
	if (o.timeout_ms <= 0)
		die("bad timeout %d\n", o.timeout_ms);
}

/* ------------------------------------------------------------------ */
/* bit writer (MSB first)                                             */
/* ------------------------------------------------------------------ */

struct bw {
	uint8_t *buf;
	size_t   cap;		/* allocated bytes */
	size_t   nbits;
};

static void bw_init(struct bw *w, size_t cap_bytes)
{
	w->buf = calloc(1, cap_bytes);
	if (!w->buf)
		die("out of memory\n");
	w->cap = cap_bytes;
	w->nbits = 0;
}

static void bw_bits(struct bw *w, size_t extra_bits)
{
	size_t need = (w->nbits + extra_bits + 7) / 8;
	if (need > w->cap) {
		size_t old = w->cap;
		while (need > w->cap)
			w->cap = w->cap ? w->cap * 2 : 1024;
		w->buf = realloc(w->buf, w->cap);
		if (!w->buf)
			die("out of memory\n");
		memset(w->buf + old, 0, w->cap - old);
	}
}

static void bw_u(struct bw *w, int n, uint32_t v)
{
	size_t base = w->nbits;
	int i;
	bw_bits(w, n);
	for (i = n - 1; i >= 0; i--) {
		size_t bit = base + (n - 1 - i);
		if ((v >> i) & 1)
			w->buf[bit / 8] |= 0x80 >> (bit % 8);
	}
	w->nbits = base + n;
}

static void bw_ue(struct bw *w, uint32_t v)
{
	uint32_t s = v + 1;
	int n = 0;
	while (s >> n)
		n++;
	bw_u(w, n - 1, 0);
	bw_u(w, n, s);
}

static void bw_se(struct bw *w, int32_t v)
{
	if (v <= 0)
		bw_ue(w, (uint32_t)(-2 * v));
	else
		bw_ue(w, (uint32_t)(2 * v - 1));
}

/* append raw bytes at the current (possibly unaligned) bit position */
static void bw_bytes(struct bw *w, const uint8_t *d, size_t n)
{
	size_t i;
	bw_bits(w, n * 8);
	for (i = 0; i < n; i++)
		bw_u(w, 8, d[i]);
}

/* rbsp trailing bits */
static void bw_trailing(struct bw *w)
{
	bw_bits(w, 8);
	w->buf[w->nbits / 8] |= 0x80 >> (w->nbits % 8);
	w->nbits++;
	while (w->nbits % 8)
		w->nbits++;
}

/* pad with zero bits to the byte grid, return byte count */
static size_t bw_finish(struct bw *w)
{
	while (w->nbits % 8)
		w->nbits++;
	return w->nbits / 8;
}

/* ------------------------------------------------------------------ */
/* NAL packaging (SPS/PPS/slice headers, EPB, Annex-B)                */
/* ------------------------------------------------------------------ */

struct stream_cfg {
	int mbw, mbh;		/* picture size in macroblocks */
	int W, H;		/* requested (uncropped) size */
	int qp;
	int level_idc;
	int fps;
	int log2_mfn;		/* log2_max_frame_num */
	int crop_r, crop_b;	/* chroma crop units */
};

static void write_nal(FILE *f, int nal_ref_idc, int nal_type,
		      const uint8_t *rbsp, size_t len)
{
	static const uint8_t sc[4] = { 0, 0, 0, 1 };
	uint8_t hdr = (uint8_t)((nal_ref_idc << 5) | nal_type);
	int zeros = 0;
	size_t i;

	fwrite(sc, 1, 4, f);
	fputc(hdr, f);
	/* emulation prevention */
	for (i = 0; i < len; i++) {
		uint8_t b = rbsp[i];
		if (zeros >= 2 && b <= 3) {
			fputc(3, f);
			zeros = 0;
		}
		fputc(b, f);
		if (b == 0)
			zeros++;
		else
			zeros = 0;
	}
}

static size_t make_sps(const struct stream_cfg *c, uint8_t *rbsp)
{
	struct bw w;

	bw_init(&w, 64);
	bw_u(&w, 8, 66);			/* profile_idc: baseline */
	bw_u(&w, 8, 0);				/* constraint flags + reserved */
	bw_u(&w, 8, (uint32_t)c->level_idc);
	bw_ue(&w, 0);				/* seq_parameter_set_id */
	bw_ue(&w, (uint32_t)(c->log2_mfn - 4));	/* log2_max_frame_num_minus4 */
	bw_ue(&w, 2);				/* pic_order_cnt_type = 2 */
	bw_ue(&w, 1);				/* max_num_ref_frames */
	bw_u(&w, 1, 0);				/* gaps_in_frame_num_value_allowed */
	bw_ue(&w, (uint32_t)(c->mbw - 1));
	bw_ue(&w, (uint32_t)(c->mbh - 1));
	bw_u(&w, 1, 1);				/* frame_mbs_only_flag */
	bw_u(&w, 1, 1);				/* direct_8x8_inference_flag */
	if (c->crop_r || c->crop_b) {
		bw_u(&w, 1, 1);			/* frame_cropping_flag */
		bw_ue(&w, 0);			/* left */
		bw_ue(&w, (uint32_t)c->crop_r);	/* right */
		bw_ue(&w, 0);			/* top */
		bw_ue(&w, (uint32_t)c->crop_b);	/* bottom */
	} else {
		bw_u(&w, 1, 0);
	}
	/* VUI: timing information so players report the requested fps */
	bw_u(&w, 1, 1);				/* vui_parameters_present_flag */
	bw_u(&w, 1, 0);				/* aspect_ratio_info_present */
	bw_u(&w, 1, 0);				/* overscan_info_present */
	bw_u(&w, 1, 0);				/* video_signal_type_present */
	bw_u(&w, 1, 0);				/* chroma_loc_info_present */
	bw_u(&w, 1, 1);				/* timing_info_present_flag */
	/* num_units_in_tick / time_scale are u(32), not ue(v): the H.264
	 * spec's allowed range exceeds what exp-golomb can encode, and both
	 * x264 and ffmpeg use fixed 32-bit fields here */
	bw_u(&w, 32, 1);			/* num_units_in_tick */
	bw_u(&w, 32, (uint32_t)(2 * o.fps));	/* time_scale */
	bw_u(&w, 1, 1);				/* fixed_frame_rate_flag */
	bw_u(&w, 1, 0);				/* nal_hrd_parameters_present */
	bw_u(&w, 1, 0);				/* vcl_hrd_parameters_present */
	bw_u(&w, 1, 0);				/* pic_struct_present */
	bw_u(&w, 1, 0);				/* bitstream_restriction */
	bw_trailing(&w);
	memcpy(rbsp, w.buf, (w.nbits + 7) / 8);
	size_t n = bw_finish(&w);
	free(w.buf);
	return n;
}

static size_t make_pps(const struct stream_cfg *c, uint8_t *rbsp)
{
	struct bw w;

	bw_init(&w, 32);
	bw_ue(&w, 0);				/* pic_parameter_set_id */
	bw_ue(&w, 0);				/* seq_parameter_set_id */
	bw_u(&w, 1, 0);				/* entropy_coding_mode: CAVLC */
	bw_u(&w, 1, 0);				/* bottom_field_pic_order_in_frame_present */
	bw_ue(&w, 0);				/* num_slice_groups_minus1 */
	bw_ue(&w, 0);				/* num_ref_idx_l0_default_active_minus1 */
	bw_ue(&w, 0);				/* num_ref_idx_l1_default_active_minus1 */
	bw_u(&w, 1, 0);				/* weighted_pred_flag */
	bw_u(&w, 2, 0);				/* weighted_bipred_idc */
	bw_se(&w, (int32_t)c->qp - 26);		/* pic_init_qp_minus26 */
	bw_se(&w, 0);				/* pic_init_qs_minus26 */
	bw_se(&w, 0);				/* chroma_qp_index_offset */
	bw_u(&w, 1, 0);				/* deblocking_filter_control_present */
	bw_u(&w, 1, 0);				/* constrained_intra_pred */
	bw_u(&w, 1, 0);				/* redundant_pic_cnt_present */
	bw_trailing(&w);
	memcpy(rbsp, w.buf, (w.nbits + 7) / 8);
	size_t n = bw_finish(&w);
	free(w.buf);
	return n;
}

/*
 * Assemble one slice NAL: slice header bits + the core's macroblock-layer
 * bits, emulation-prevention applied, Annex-B start code prepended.
 */
static void write_slice_nal(FILE *f, const struct stream_cfg *c,
			    int frame_num, int intra, int idr_pic_id,
			    const uint8_t *core, size_t core_len)
{
	struct bw w;

	bw_init(&w, 64);
	bw_ue(&w, 0);				/* first_mb_in_slice */
	bw_ue(&w, intra ? 7 : 5);		/* slice_type: all-I / all-P */
	bw_ue(&w, 0);				/* pic_parameter_set_id */
	bw_u(&w, c->log2_mfn, (uint32_t)frame_num);
	if (intra) {
		bw_ue(&w, (uint32_t)idr_pic_id);
		bw_u(&w, 1, 0);			/* no_output_of_prior_pics */
		bw_u(&w, 1, 0);			/* long_term_reference_flag */
	} else {
		bw_u(&w, 1, 0);			/* num_ref_idx_active_override_flag */
		bw_u(&w, 1, 0);			/* ref_pic_list_modification_flag_l0 */
		bw_u(&w, 1, 0);			/* adaptive_ref_pic_marking_mode_flag */
	}
	bw_se(&w, 0);				/* slice_qp_delta */
	bw_bytes(&w, core, core_len);
	size_t n = bw_finish(&w);
	write_nal(f, intra ? 3 : 2, intra ? 5 : 1, w.buf, n);
	free(w.buf);
}

/*
 * Undo the wrapper's 64-bit big-endian word packing: in DDR the
 * bitstream is stored with each 8-byte group byte-reversed (the
 * trailing partial group likewise). core_len is BSLEN from the
 * hardware, core must hold core_len bytes.
 */
static void deroll_bs(const uint8_t *ddr_bs, size_t len, uint8_t *core)
{
	size_t off;

	for (off = 0; off + 8 <= len; off += 8) {
		int i;
		for (i = 0; i < 8; i++)
			core[off + i] = ddr_bs[off + 7 - i];
	}
	while (off < len) {
		size_t tail = len - off, i;
		for (i = 0; i < tail; i++)
			core[off + i] = ddr_bs[off + tail - 1 - i];
		off += tail;
	}
}

/* ------------------------------------------------------------------ */
/* input sources: raw I420 file or built-in rolling color bars        */
/* ------------------------------------------------------------------ */

static uint8_t *src_y, *src_u, *src_v;	/* planar I420, padded size */
static FILE *src_f;
static int src_cw, src_ch;
static int pattern_nframes;

static void bars(uint8_t *y, uint8_t *u, uint8_t *v, int W, int H, int frm)
{
	static const int by[8] = { 180, 162, 131, 112, 84, 65, 35, 16 };
	static const int bu[8] = { 128, 44, 160, 72, 184, 100, 212, 128 };
	static const int bv[8] = { 128, 142, 16, 58, 198, 212, 114, 128 };
	int lsh = frm * 16, csh = frm * 8;	/* luma/chroma shift, px */
	int x, yy;

	for (yy = 0; yy < H; yy++)
		for (x = 0; x < W; x++) {
			int b = ((x + lsh) % W) / (W / 8);
			if (b > 7)
				b = 7;
			y[yy * W + x] = (uint8_t)by[b];
		}
	for (yy = 0; yy < src_ch; yy++)
		for (x = 0; x < src_cw; x++) {
			int b = ((x + csh) % src_cw) / (src_cw / 8);
			if (b > 7)
				b = 7;
			u[yy * src_cw + x] = (uint8_t)bu[b];
			v[yy * src_cw + x] = (uint8_t)bv[b];
		}
}

/* returns 0 when no more frames */
static int next_frame(int frm)
{
	if (src_f) {
		/* read the three I420 planes into their own buffers; a single
		 * fread() of W*H*3/2 into src_y would overflow it by H*W/2 */
		size_t ysz = (size_t)o.width * o.height;
		size_t csz = (size_t)src_cw * src_ch;

		if (fread(src_y, 1, ysz, src_f) != ysz ||
		    fread(src_u, 1, csz, src_f) != csz ||
		    fread(src_v, 1, csz, src_f) != csz) {
			if (!frm)
				fprintf(stderr,
					"input shorter than one %dx%d I420 frame (%zu bytes)\n",
					o.width, o.height, ysz + 2 * csz);
			return 0;
		}
		return 1;
	}
	bars(src_y, src_u, src_v, o.width, o.height, frm);
	return frm < pattern_nframes;
}

/*
 * Convert planar I420 to the wrapper's MB-tiled input layout:
 * per MB 384 bytes = 32 luma beats (row-major, MSB-first pixels)
 * + 16 chroma beats (U/V byte-interleaved), see tb_xk264_axi_640.v.
 */
static void planar_to_tiled(const uint8_t *y, const uint8_t *u,
			    const uint8_t *v, uint8_t *tiled)
{
	int mbw = (o.width + 15) / 16, mbh = (o.height + 15) / 16;
	int mbx, mby, r, c;

	for (mby = 0; mby < mbh; mby++)
		for (mbx = 0; mbx < mbw; mbx++) {
			uint8_t *mb = tiled + (size_t)(mby * mbw + mbx) * BYTES_PER_MB;
			for (r = 0; r < 16; r++)
				for (c = 0; c < 16; c++) {
					int sx = mbx * 16 + c, sy = mby * 16 + r;
					if (sx >= o.width)  sx = o.width - 1;
					if (sy >= o.height) sy = o.height - 1;
					mb[(r * 2 + (c >> 3)) * 8 + 7 - (c & 7)] =
						y[sy * o.width + sx];
				}
			for (r = 0; r < 8; r++)
				for (c = 0; c < 8; c++) {
					int sx = mbx * 8 + c, sy = mby * 8 + r;
					int off;
					if (sx >= src_cw) sx = src_cw - 1;
					if (sy >= src_ch) sy = src_ch - 1;
					off = 256 + (r * 2 + (c >> 2)) * 8;
					mb[off + 7 - 2 * (c & 3)] = u[sy * src_cw + sx];
					mb[off + 6 - 2 * (c & 3)] = v[sy * src_cw + sx];
				}
		}
}

/* ------------------------------------------------------------------ */
/* encode loop                                                        */
/* ------------------------------------------------------------------ */

static volatile sig_atomic_t stop_req;

static void on_sigint(int sig)
{
	(void)sig;
	stop_req = 1;
}

static uint64_t now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

static void sleep_until(uint64_t deadline)
{
	struct timespec ts;
	ts.tv_sec = deadline / 1000000000ull;
	ts.tv_nsec = deadline % 1000000000ull;
	while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) &&
	       errno == EINTR)
		;
}

int main(int argc, char **argv)
{
	struct stream_cfg c;
	struct xk264_info info = {};
	uint8_t sps[64], pps[32];
	size_t sps_len, pps_len;
	uint8_t *core = NULL, *tiled = NULL;
	size_t core_cap = 0;
	FILE *out;
	int fd = -1, mb_total, frm, nout = 0;
	uint8_t *map_yuv = NULL, *map_bs = NULL;
	uint64_t t_first = 0;
	uint64_t total_bytes = 0;
	int idr_pic_id = 0, frames_since_idr = 0, frame_num = 0;
	int log2_mfn = 4;

	parse_opts(argc, argv);
	signal(SIGINT, on_sigint);
	signal(SIGTERM, on_sigint);

	c.mbw = (o.width + 15) / 16;
	c.mbh = (o.height + 15) / 16;
	c.W = o.width;
	c.H = o.height;
	c.qp = o.qp;
	c.level_idc = o.level_idc;
	c.fps = o.fps;
	c.crop_r = (c.mbw * 16 - o.width) / 2;
	c.crop_b = (c.mbh * 16 - o.height) / 2;
	mb_total = c.mbw * c.mbh;

	while ((1u << log2_mfn) <= (uint32_t)(o.gop - 1))
		log2_mfn++;
	c.log2_mfn = log2_mfn;

	if (!o.nframes) {
		if (o.input || o.sim_dir)
			o.nframes = 1 << 30;		/* until EOF */
		else
			o.nframes = 60;			/* pattern default */
	}
	pattern_nframes = o.nframes;

	printf("xk264_app: %dx%d (%dx%d MB) %dfps gop=%d qp=%d%s\n",
	       o.width, o.height, c.mbw, c.mbh, o.fps, o.gop, o.qp,
	       (c.crop_r || c.crop_b) ? " (cropped to display size)" : "");
	printf("output: %s, %s, %s\n", o.output,
	       o.nframes == (1 << 30) ? "until EOF" : "N frames",
	       o.sim_dir ? "offline (simulation dumps)" :
	       (o.input ? o.input : "built-in rolling bars"));

	/* ---- input source ---- */
	src_cw = (o.width + 1) / 2;
	src_ch = (o.height + 1) / 2;
	src_y = malloc((size_t)o.width * o.height);
	src_u = malloc((size_t)src_cw * src_ch);
	src_v = malloc((size_t)src_cw * src_ch);
	tiled = malloc((size_t)mb_total * BYTES_PER_MB);
	if (!src_y || !src_u || !src_v || !tiled)
		die("out of memory\n");
	if (o.input) {
		src_f = fopen(o.input, "rb");
		if (!src_f)
			die("cannot open input %s: %s\n", o.input, strerror(errno));
	}

	/* ---- hardware / offline ---- */
	if (!o.sim_dir) {
		struct xk264_param p = {
			.mb_x_total = (uint16_t)c.mbw,
			.mb_y_total = (uint16_t)c.mbh,
			.qp = (uint8_t)o.qp,
		};

		fd = open(DEV_PATH, O_RDWR);
		if (fd < 0)
			die("cannot open %s: %s\n", DEV_PATH, strerror(errno));
		if (ioctl(fd, XK264_IOC_G_INFO, &info))
			die("G_INFO: %s\n", strerror(errno));
		if (info.hw_id != 0x02640001u)
			die("bad hw id %#x\n", info.hw_id);
		if (mb_total > (int)info.max_mbs)
			die("picture too large: %d MBs > max %d\n",
			    mb_total, info.max_mbs);
		if (ioctl(fd, XK264_IOC_S_PARAM, &p))
			die("S_PARAM: %s\n", strerror(errno));

		map_yuv = mmap(NULL, info.mmap_len[0], PROT_READ | PROT_WRITE,
			       MAP_SHARED, fd, info.mmap_off[0]);
		map_bs = mmap(NULL, info.mmap_len[1], PROT_READ | PROT_WRITE,
			      MAP_SHARED, fd, info.mmap_off[1]);
		if (map_yuv == MAP_FAILED || map_bs == MAP_FAILED)
			die("mmap: %s\n", strerror(errno));
	}

	sps_len = make_sps(&c, sps);
	pps_len = make_pps(&c, pps);

	out = fopen(o.output, "wb");
	if (!out)
		die("cannot open output %s: %s\n", o.output, strerror(errno));

	uint64_t period = 1000000000ull / (uint64_t)o.fps;
	uint64_t next_deadline = now_ns() + period;

	for (frm = 0; frm < o.nframes && !stop_req; frm++) {
		struct xk264_encode enc = { .timeout_ms = (uint32_t)o.timeout_ms };
		int intra = (frm % o.gop) == 0;
		uint64_t t0, t1;
		size_t core_len;
		char path[512];
		FILE *bf;

		if (!next_frame(frm))
			break;

		t0 = now_ns();

		if (o.sim_dir) {
			snprintf(path, sizeof(path), "%s/bs_frame%d.bin",
				 o.sim_dir, frm);
			bf = fopen(path, "rb");
			if (!bf) {
				if (frm == 0)
					die("no %s\n", path);
				break;
			}
			fseek(bf, 0, SEEK_END);
			core_len = (size_t)ftell(bf);
			fseek(bf, 0, SEEK_SET);
			if (core_len > core_cap) {
				core_cap = core_len + 65536;
				core = realloc(core, core_cap);
				if (!core)
					die("out of memory\n");
			}
			uint8_t *raw = malloc(core_len);
			if (!raw)
				die("out of memory\n");
			if (fread(raw, 1, core_len, bf) != core_len)
				die("short read %s\n", path);
			fclose(bf);
			deroll_bs(raw, core_len, core);
			free(raw);
		} else {
			planar_to_tiled(src_y, src_u, src_v, map_yuv);
			enc.flags = intra ? XK264_ENC_INTRA : 0;
			if (ioctl(fd, XK264_IOC_ENCODE, &enc))
				die("frame %d ENCODE: %s\n", frm, strerror(errno));
			core_len = enc.bs_len;
			if (core_len > info.mmap_len[1])
				die("frame %d: bad bs_len %u\n", frm, enc.bs_len);
			if (core_len > core_cap) {
				core_cap = core_len + 65536;
				core = realloc(core, core_cap);
				if (!core)
					die("out of memory\n");
			}
			deroll_bs(map_bs, core_len, core);
		}

		if (intra) {
			if (frames_since_idr)
				idr_pic_id ^= 1;
			frame_num = 0;
			frames_since_idr = 0;
			write_nal(out, 3, 7, sps, sps_len);
			write_nal(out, 3, 8, pps, pps_len);
		} else {
			frame_num = frames_since_idr;
		}
		write_slice_nal(out, &c, frame_num, intra, idr_pic_id,
				core, core_len);
		frames_since_idr++;
		nout++;

		t1 = now_ns();
		if (!t_first)
			t_first = t0;
		total_bytes += core_len;
		fprintf(stderr, "frame %3d: %s core=%zu B, %.1f ms%s\n",
			frm, intra ? "IDR" : " P ", core_len,
			(double)(t1 - t0) / 1e6,
			stop_req ? " (stopping)" : "");

		if (o.pace && !o.sim_dir) {
			next_deadline += period;
			if (now_ns() < next_deadline)
				sleep_until(next_deadline);
			else
				next_deadline = now_ns();	/* fell behind */
		}
	}

	fclose(out);
	if (fd >= 0)
		close(fd);

	double secs = (double)(now_ns() - t_first) / 1e9;
	if (nout) {
		printf("wrote %d frame(s), %llu core bytes to %s\n",
		       nout, (unsigned long long)total_bytes, o.output);
		printf("average bitrate: %.1f kbps (%.2f kB/frame)\n",
		       (double)total_bytes * 8 * o.fps / nout / 1000.0,
		       (double)total_bytes / nout / 1024.0);
		if (secs > 0 && !o.sim_dir)
			printf("achieved frame rate: %.1f fps (requested %d)\n",
			       (nout - 1) / secs, o.fps);
	} else {
		fprintf(stderr, "no frame was encoded\n");
	}
	return nout ? 0 : 1;
}
