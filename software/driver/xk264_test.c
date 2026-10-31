// SPDX-License-Identifier: GPL-2.0
/*
 * xk264_test - encode one intra frame + one inter frame through
 * /dev/xk264 and dump the bitstream to a file.
 *
 * Build: ${CROSS}gcc xk264_test.c -o xk264_test
 * Run:   ./xk264_test out.h264
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>

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

#define MBX 4
#define MBY 4

static void fill_frame(uint8_t *yuv, int frame)
{
	int mb, t, k;

	/* MB-tiled order: per MB 32 luma beats (16x16) then 16 UV beats */
	for (mb = 0; mb < MBX * MBY; mb++) {
		for (t = 0; t < 32; t++)
			for (k = 0; k < 8; k++)
				yuv[mb * 384 + t * 8 + k] =
					(mb * 3 + t * 5 + k * 7 + frame * 23) & 0xff;
		for (t = 0; t < 16; t++)
			for (k = 0; k < 8; k++)
				yuv[mb * 384 + 256 + t * 8 + k] =
					(mb * 11 + t * 3 + k + frame * 31) & 0xff;
	}
}

int main(int argc, char **argv)
{
	struct xk264_info info;
	struct xk264_param param = { .mb_x_total = MBX, .mb_y_total = MBY, .qp = 28 };
	struct xk264_encode enc = { .timeout_ms = 10000 };
	uint8_t *yuv, *bs;
	FILE *f;
	int fd, frame;

	if (argc < 2) {
		fprintf(stderr, "usage: %s <out.h264>\n", argv[0]);
		return 1;
	}

	fd = open("/dev/xk264", O_RDWR);
	if (fd < 0) { perror("open /dev/xk264"); return 1; }

	if (ioctl(fd, XK264_IOC_G_INFO, &info) ||
	    info.hw_id != 0x02640001u) {
		fprintf(stderr, "bad hw id %#x\n", info.hw_id);
		return 1;
	}

	yuv = mmap(NULL, info.mmap_len[0], PROT_READ | PROT_WRITE,
		   MAP_SHARED, fd, info.mmap_off[0]);
	bs = mmap(NULL, info.mmap_len[1], PROT_READ | PROT_WRITE,
		  MAP_SHARED, fd, info.mmap_off[1]);
	if (yuv == MAP_FAILED || bs == MAP_FAILED) {
		perror("mmap"); return 1;
	}

	if (ioctl(fd, XK264_IOC_S_PARAM, &param)) {
		perror("S_PARAM"); return 1;
	}

	f = fopen(argv[1], "wb");

	for (frame = 0; frame < 2; frame++) {
		fill_frame(yuv, frame);
		enc.flags = (frame == 0) ? XK264_ENC_INTRA : 0;
		if (ioctl(fd, XK264_IOC_ENCODE, &enc)) {
			perror("ENCODE"); return 1;
		}
		printf("frame %d: %u bytes\n", frame, enc.bs_len);
		fwrite(bs, 1, enc.bs_len, f);
	}

	fclose(f);
	close(fd);
	return 0;
}
