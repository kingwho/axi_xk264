// SPDX-License-Identifier: GPL-2.0
/*
 * xk264-v4l2-app - userspace demo for the xk264 V4L2 mem2mem encoder
 * driver (xk264-v4l2.ko, /dev/videoN).
 *
 * Encodes a raw I420 input file (or a built-in rolling color-bar test
 * pattern) through the standard V4L2 memory-to-memory interface and
 * writes a directly playable Annex-B .h264 stream:
 *
 *   OUTPUT  queue: V4L2_PIX_FMT_YUV420M (3 planes, I420)
 *   CAPTURE queue: V4L2_PIX_FMT_H264    (Annex-B, SPS/PPS per IDR)
 *
 * Frame rate, GOP and QP map to V4L2 controls:
 *   VIDIOC_S_PARM (timeperframe), V4L2_CID_MPEG_VIDEO_GOP_SIZE,
 *   V4L2_CID_MPEG_VIDEO_H264_MIN_QP, V4L2_CID_MPEG_VIDEO_H264_LEVEL.
 *
 * Build (target, PetaLinux 2020.2 / Zynq-7000):
 *     make CROSS=arm-linux-gnueabihf- xk264-v4l2-app
 *
 * Run on target:
 *     ./xk264-v4l2-app -d /dev/video0 -W 640 -H 480 -f 30 -g 30 -q 28 \
 *                      -i in.yuv -o out.h264
 *     ffplay out.h264
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
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/videodev2.h>

#define DEV_PATH	"/dev/video0"
#define MAX_MBS		((1920u / 16u) * (1080u / 16u))
#define N_OUT_BUFS	2
#define N_CAP_BUFS	2
#define MAX_PLANES	3

struct opts {
	int		width, height;
	int		fps;
	int		gop;
	int		qp;
	int		level_idc;
	int		nframes;
	const char	*device;
	const char	*input;
	const char	*output;
};

static struct opts o = {
	.width = 640, .height = 480,
	.fps = 30, .gop = 30, .qp = 28, .level_idc = 40,
	.nframes = 0,
	.device = DEV_PATH, .input = NULL, .output = "out.h264",
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
"usage: xk264-v4l2-app [options]\n"
"  -d, --device PATH  video node (default %s)\n"
"  -W, --width N      picture width in pixels  (default 640)\n"
"  -H, --height N     picture height in pixels (default 480)\n"
"  -f, --fps N        frame rate, set via VIDIOC_S_PARM (default 30)\n"
"  -g, --gop N        IDR interval in frames, 1 = all-intra (default 30)\n"
"  -q, --qp N         quantization parameter 0..51 (default 28)\n"
"  -n, --frames N     number of frames (default: input EOF,\n"
"                     60 for the built-in test pattern)\n"
"  -i, --input FILE   raw I420 planar input file (default: test pattern)\n"
"  -o, --output FILE  output Annex-B H.264 file (default out.h264)\n"
"      --level IDC    SPS level_idc (default 40)\n"
"  -h, --help         show this help\n", DEV_PATH);
	exit(1);
}

static void parse(int argc, char **argv)
{
	static const struct option longopts[] = {
		{ "device", required_argument, NULL, 'd' },
		{ "width",  required_argument, NULL, 'W' },
		{ "height", required_argument, NULL, 'H' },
		{ "fps",    required_argument, NULL, 'f' },
		{ "gop",    required_argument, NULL, 'g' },
		{ "qp",     required_argument, NULL, 'q' },
		{ "frames", required_argument, NULL, 'n' },
		{ "input",  required_argument, NULL, 'i' },
		{ "output", required_argument, NULL, 'o' },
		{ "level",  required_argument, NULL, 'L' },
		{ "help",   no_argument,       NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int c;

	while ((c = getopt_long(argc, argv, "d:W:H:f:g:q:n:i:o:h", longopts, NULL)) != -1) {
		switch (c) {
		case 'd': o.device = optarg; break;
		case 'W': o.width  = atoi(optarg); break;
		case 'H': o.height = atoi(optarg); break;
		case 'f': o.fps    = atoi(optarg); break;
		case 'g': o.gop    = atoi(optarg); break;
		case 'q': o.qp     = atoi(optarg); break;
		case 'n': o.nframes = atoi(optarg); break;
		case 'i': o.input  = optarg; break;
		case 'o': o.output = optarg; break;
		case 'L': o.level_idc = atoi(optarg); break;
		default:  usage();
		}
	}

	if (o.width < 32 || o.height < 32 || (o.width & 1) || (o.height & 1))
		die("bad size %dx%d (even, >= 32)\n", o.width, o.height);
	if ((o.width / 16) * (o.height / 16) > (int)MAX_MBS)
		die("picture too large: %dx%d (%d MBs, max %u)\n",
		    o.width, o.height,
		    (o.width / 16) * (o.height / 16), MAX_MBS);
	if (o.fps <= 0 || o.fps > 240)
		die("bad fps %d\n", o.fps);
	if (o.gop <= 0)
		die("bad gop %d\n", o.gop);
	if (o.qp < 0 || o.qp > 51)
		die("bad qp %d\n", o.qp);
}

static int xioctl(int fd, unsigned long req, void *arg)
{
	int r;

	do {
		r = ioctl(fd, req, arg);
	} while (r < 0 && errno == EINTR);
	return r;
}

static int level_to_enum(int idc)
{
	switch (idc) {
	case 10: return V4L2_MPEG_VIDEO_H264_LEVEL_1_0;
	case 11: return V4L2_MPEG_VIDEO_H264_LEVEL_1_1;
	case 12: return V4L2_MPEG_VIDEO_H264_LEVEL_1_2;
	case 13: return V4L2_MPEG_VIDEO_H264_LEVEL_1_3;
	case 20: return V4L2_MPEG_VIDEO_H264_LEVEL_2_0;
	case 21: return V4L2_MPEG_VIDEO_H264_LEVEL_2_1;
	case 22: return V4L2_MPEG_VIDEO_H264_LEVEL_2_2;
	case 30: return V4L2_MPEG_VIDEO_H264_LEVEL_3_0;
	case 31: return V4L2_MPEG_VIDEO_H264_LEVEL_3_1;
	case 32: return V4L2_MPEG_VIDEO_H264_LEVEL_3_2;
	case 40: return V4L2_MPEG_VIDEO_H264_LEVEL_4_0;
	case 41: return V4L2_MPEG_VIDEO_H264_LEVEL_4_1;
	case 42: return V4L2_MPEG_VIDEO_H264_LEVEL_4_2;
	case 50: return V4L2_MPEG_VIDEO_H264_LEVEL_5_0;
	case 51: return V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
	default: return V4L2_MPEG_VIDEO_H264_LEVEL_4_0;
	}
}

/* ------------------------------------------------------------------ */
/* V4L2 buffers                                                       */
/* ------------------------------------------------------------------ */

struct mp_buf {
	void	*start[MAX_PLANES];
	size_t	len[MAX_PLANES];
};

static struct mp_buf out_bufs[N_OUT_BUFS];
static struct mp_buf cap_bufs[N_CAP_BUFS];
static unsigned out_nplanes = 3, cap_nplanes = 1;
static size_t out_used[MAX_PLANES];	/* bytesused per OUTPUT plane */

/* per-OUTPUT-buffer QBUF timestamp (ms, CLOCK_MONOTONIC) for latency */
static double out_qtime[N_OUT_BUFS];

static double now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}

static void reqbufs(int fd, unsigned type, unsigned count)
{
	struct v4l2_requestbuffers req;

	memset(&req, 0, sizeof(req));
	req.count = count;
	req.type = type;
	req.memory = V4L2_MEMORY_MMAP;
	if (xioctl(fd, VIDIOC_REQBUFS, &req))
		die("REQBUFS type %u: %s\n", type, strerror(errno));
	if (req.count != count)
		die("REQBUFS type %u: got %u of %u buffers\n",
		    type, req.count, count);
}

static void map_bufs(int fd, unsigned type, struct mp_buf *bufs,
		     unsigned count, unsigned nplanes)
{
	unsigned i, p;

	for (i = 0; i < count; i++) {
		struct v4l2_plane planes[MAX_PLANES];
		struct v4l2_buffer buf;

		memset(planes, 0, sizeof(planes));
		memset(&buf, 0, sizeof(buf));
		buf.type = type;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		buf.length = nplanes;
		buf.m.planes = planes;
		if (xioctl(fd, VIDIOC_QUERYBUF, &buf))
			die("QUERYBUF type %u idx %u: %s\n",
			    type, i, strerror(errno));
		for (p = 0; p < nplanes; p++) {
			bufs[i].len[p] = planes[p].length;
			bufs[i].start[p] = mmap(NULL, planes[p].length,
						PROT_READ | PROT_WRITE,
						MAP_SHARED, fd,
						planes[p].m.mem_offset);
			if (bufs[i].start[p] == MAP_FAILED)
				die("mmap type %u idx %u plane %u: %s\n",
				    type, i, p, strerror(errno));
		}
	}
}

static void unmap_bufs(struct mp_buf *bufs, unsigned count, unsigned nplanes)
{
	unsigned i, p;

	for (i = 0; i < count; i++)
		for (p = 0; p < nplanes; p++)
			if (bufs[i].start[p] && bufs[i].start[p] != MAP_FAILED)
				munmap(bufs[i].start[p], bufs[i].len[p]);
}

static int do_qbuf(int fd, unsigned type, unsigned index, unsigned nplanes,
		   const size_t *used)
{
	struct v4l2_plane planes[MAX_PLANES];
	struct v4l2_buffer buf;
	unsigned p;

	memset(planes, 0, sizeof(planes));
	memset(&buf, 0, sizeof(buf));
	buf.type = type;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = index;
	buf.length = nplanes;
	buf.m.planes = planes;
	if (used)
		for (p = 0; p < nplanes; p++)
			planes[p].bytesused = (uint32_t)used[p];
	return xioctl(fd, VIDIOC_QBUF, &buf);
}

static int do_dqbuf(int fd, unsigned type, unsigned nplanes,
		    struct v4l2_plane *planes)
{
	struct v4l2_buffer buf;

	memset(planes, 0, sizeof(*planes) * nplanes);
	memset(&buf, 0, sizeof(buf));
	buf.type = type;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.length = nplanes;
	buf.m.planes = planes;
	if (xioctl(fd, VIDIOC_DQBUF, &buf))
		return -errno;
	return (int)buf.index;
}

/* ------------------------------------------------------------------ */
/* input source: raw I420 file or built-in rolling color bars         */
/* ------------------------------------------------------------------ */

static FILE *src_f;

static void bars(uint8_t *y, uint8_t *u, uint8_t *v, int W, int H, int frm)
{
	static const int by[8] = { 180, 162, 131, 112, 84, 65, 35, 16 };
	static const int bu[8] = { 128, 44, 160, 72, 184, 100, 212, 128 };
	static const int bv[8] = { 128, 142, 16, 58, 198, 212, 114, 128 };
	int lsh = frm * 16, csh = frm * 8;	/* luma/chroma shift, px */
	int x, yy;

	for (yy = 0; yy < H; yy++)
		for (x = 0; x < W; x++) {
			int col = ((x + lsh) * 8) / W;
			y[yy * W + x] = (uint8_t)by[col];
		}
	for (yy = 0; yy < H / 2; yy++)
		for (x = 0; x < W / 2; x++) {
			int col = ((x + csh) * 8) / (W / 2);
			u[yy * (W / 2) + x] = (uint8_t)bu[col];
			v[yy * (W / 2) + x] = (uint8_t)bv[col];
		}
}

/* fill the three OUTPUT planes of one buffer; returns 0 or EOF */
static int fill_frame(struct mp_buf *b, int frm)
{
	uint8_t *y = b->start[0], *u = b->start[1], *v = b->start[2];
	size_t ysz = out_used[0], csz = out_used[1];

	if (src_f) {
		if (fread(y, 1, ysz, src_f) != ysz)
			return -1;
		if (fread(u, 1, csz, src_f) != csz)
			return -1;
		if (fread(v, 1, csz, src_f) != csz)
			return -1;
	} else {
		bars(y, u, v, o.width, o.height, frm);
	}
	return 0;
}

/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	struct v4l2_streamparm parm;
	struct v4l2_ext_control ctrl[3];
	struct v4l2_ext_controls ctrls;
	FILE *out;
	unsigned type;
	int fd, total, done, eof;
	unsigned out_next = 0;
	double t_first = 0, t_last = 0;
	double sum_lat = 0, min_lat = 1e9, max_lat = 0;

	parse(argc, argv);

	if (o.input) {
		src_f = fopen(o.input, "rb");
		if (!src_f)
			die("open %s: %s\n", o.input, strerror(errno));
	} else if (o.nframes <= 0) {
		o.nframes = 60;
	}

	out = fopen(o.output, "wb");
	if (!out)
		die("open %s: %s\n", o.output, strerror(errno));

	fd = open(o.device, O_RDWR);
	if (fd < 0)
		die("open %s: %s\n", o.device, strerror(errno));

	memset(&cap, 0, sizeof(cap));
	if (xioctl(fd, VIDIOC_QUERYCAP, &cap))
		die("QUERYCAP: %s\n", strerror(errno));
	if (!(cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE))
		die("%s is not a V4L2 M2M_MPLANE device (caps %#x)\n",
		    o.device, cap.capabilities);

	/* OUTPUT: I420 multiplanar */
	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fmt.fmt.pix_mp.width = (uint32_t)o.width;
	fmt.fmt.pix_mp.height = (uint32_t)o.height;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420M;
	fmt.fmt.pix_mp.num_planes = 3;
	if (xioctl(fd, VIDIOC_S_FMT, &fmt))
		die("S_FMT OUTPUT: %s\n", strerror(errno));

	out_nplanes = fmt.fmt.pix_mp.num_planes;
	if (out_nplanes != 3)
		die("driver returned %u OUTPUT planes (expected 3)\n",
		    out_nplanes);
	out_used[0] = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
	out_used[1] = fmt.fmt.pix_mp.plane_fmt[1].sizeimage;
	out_used[2] = fmt.fmt.pix_mp.plane_fmt[2].sizeimage;

	/* CAPTURE: H.264 Annex-B */
	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	if (xioctl(fd, VIDIOC_S_FMT, &fmt))
		die("S_FMT CAPTURE: %s\n", strerror(errno));
	cap_nplanes = fmt.fmt.pix_mp.num_planes;

	/* frame rate */
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	parm.parm.output.timeperframe.numerator = 1;
	parm.parm.output.timeperframe.denominator = (uint32_t)o.fps;
	if (xioctl(fd, VIDIOC_S_PARM, &parm))
		die("S_PARM: %s\n", strerror(errno));

	/* encoder controls */
	memset(ctrl, 0, sizeof(ctrl));
	ctrl[0].id = V4L2_CID_MPEG_VIDEO_GOP_SIZE;
	ctrl[0].value = o.gop;
	ctrl[1].id = V4L2_CID_MPEG_VIDEO_H264_MIN_QP;
	ctrl[1].value = o.qp;
	ctrl[2].id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
	ctrl[2].value = level_to_enum(o.level_idc);
	memset(&ctrls, 0, sizeof(ctrls));
	ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
	ctrls.count = 3;
	ctrls.controls = ctrl;
	if (xioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls))
		die("S_EXT_CTRLS: %s\n", strerror(errno));

	/* buffers */
	reqbufs(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, N_OUT_BUFS);
	reqbufs(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, N_CAP_BUFS);
	map_bufs(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, out_bufs,
		 N_OUT_BUFS, out_nplanes);
	map_bufs(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, cap_bufs,
		 N_CAP_BUFS, cap_nplanes);

	/* pre-queue all capture buffers */
	{
		unsigned i;
		for (i = 0; i < N_CAP_BUFS; i++)
			if (do_qbuf(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
				    i, cap_nplanes, NULL))
				die("QBUF capture %u: %s\n", i, strerror(errno));
	}

	type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (xioctl(fd, VIDIOC_STREAMON, &type))
		die("STREAMON OUTPUT: %s\n", strerror(errno));
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (xioctl(fd, VIDIOC_STREAMON, &type))
		die("STREAMON CAPTURE: %s\n", strerror(errno));

	printf("xk264-v4l2-app: %s %dx%d %dfps gop=%d qp=%d -> %s\n",
	       o.device, o.width, o.height, o.fps, o.gop, o.qp, o.output);

	/*
	 * Streaming loop: keep up to N_OUT_BUFS frames in flight.  Each
	 * iteration waits for one finished job (blocking dqbuf), writes
	 * the capture payload and re-queues the capture buffer.
	 */
	total = 0;
	done = 0;
	eof = 0;
	while (!eof || done < total) {
		/* feed the encoder while there are frames left */
		while (!eof && (!o.nframes || total < o.nframes) &&
		       total - done < N_OUT_BUFS) {
			if (fill_frame(&out_bufs[out_next], total)) {
				eof = 1;		/* input exhausted */
				break;
			}
			out_qtime[out_next] = now_ms();
			if (total == 0)
				t_first = out_qtime[out_next];
			if (do_qbuf(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				    out_next, out_nplanes, out_used))
				die("QBUF output %u: %s\n",
				    out_next, strerror(errno));
			out_next = (out_next + 1) % N_OUT_BUFS;
			total++;
		}

		if (done >= total)
			break;			/* nothing in flight */

		/* wait for one finished job: OUTPUT then CAPTURE */
		{
			struct v4l2_plane planes[MAX_PLANES];
			int oidx, idx;
			size_t bytes;
			double t_enc, t_end;

			oidx = do_dqbuf(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
					out_nplanes, planes);
			if (oidx < 0)
				die("DQBUF OUTPUT: %s\n", strerror(-oidx));

			idx = do_dqbuf(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
				       cap_nplanes, planes);
			if (idx < 0)
				die("DQBUF CAPTURE: %s\n", strerror(-idx));

			t_end = now_ms();
			t_enc = t_end - out_qtime[oidx];
			t_last = t_end;
			sum_lat += t_enc;
			if (t_enc < min_lat)
				min_lat = t_enc;
			if (t_enc > max_lat)
				max_lat = t_enc;

			bytes = planes[0].bytesused;
			if (fwrite(cap_bufs[idx].start[0], 1, bytes, out) !=
			    bytes)
				die("write %s: %s\n", o.output,
				    strerror(errno));

			fprintf(stderr, "frame %3d: %s %6zu B  %7.2f ms\n",
				done, (done % o.gop) ? " P " : "IDR",
				bytes, t_enc);
			done++;

			if (do_qbuf(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
				    (unsigned)idx, cap_nplanes, NULL))
				die("re-QBUF capture %d: %s\n",
				    idx, strerror(errno));
		}
	}

	if (done > 0) {
		double elapsed = (t_last - t_first) / 1000.0;
		double fps = elapsed > 0 ? done / elapsed : 0;

		printf("done: %d frames -> %s\n", done, o.output);
		printf("elapsed: %.3f s, fps: %.2f\n", elapsed, fps);
		printf("encode latency: avg %.2f ms, min %.2f ms, max %.2f ms\n",
		       sum_lat / done, min_lat, max_lat);
	} else {
		printf("done: 0 frames -> %s\n", o.output);
	}

	type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (xioctl(fd, VIDIOC_STREAMOFF, &type))
		die("STREAMOFF OUTPUT: %s\n", strerror(errno));
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (xioctl(fd, VIDIOC_STREAMOFF, &type))
		die("STREAMOFF CAPTURE: %s\n", strerror(errno));

	unmap_bufs(out_bufs, N_OUT_BUFS, out_nplanes);
	unmap_bufs(cap_bufs, N_CAP_BUFS, cap_nplanes);
	close(fd);
	fclose(out);
	if (src_f)
		fclose(src_f);
	return 0;
}
